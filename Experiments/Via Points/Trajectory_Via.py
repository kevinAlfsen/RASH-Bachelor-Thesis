import numpy as np
import config

class Trajectory:
    """
    Trajectory class using trajectory generation with via points, with constant velocity segments and 
    continous acceleration blends. For more info on this check out W.Khalils book Modeling, identification and control of robots
    
    
    Formulas and conditions are taken from W. Khalil, E. Dombre - Modeling Identification and Control of Robots (2004, Butterworth-Heinemann)
    All comments with format, [eq.no], refer to the corresponding equation from this book. 
    """
    def __init__(self, points, k_v = config.default_speed, k_a = config.default_acceleration):
        
        if isinstance(points, list) or isinstance(points, tuple):
            points = np.vstack(points)

        #Class wide variables
        self.points = points
        self.q_f = points[-1]
        
        self.num_points, self.num_joints = points.shape
        self.num_segments = self.num_points - 1 #There is one segment between each point => there is one less segment than points        
        self.segment_index = np.zeros(self.num_joints, dtype=int) #Explained in via()
        
        #Calculate distance between each point for each joint
        D = np.array([self.points[i + 1] - self.points[i] for i in range(self.num_segments)]) #Distance
        #print(f"D after init : {D}")

        #Determine the direction of each distance
        sign_D = np.zeros(D.shape) 
        mask = D != 0        
        sign_D[mask] = D[mask]/abs(D[mask]) #Left with -1 or 1

        #Velocity and acceleration needs the same shape as the distance, so that each segment has it's velocity and acceleration     
        k_v = np.tile(k_v, (self.num_segments,1)) 
        k_a = np.tile(k_a, (self.num_segments,1))
        
        #[13.46]
        #The distance must be large enough to allow the joint to reach maximum velocity with the given maximum acceleration. 
        #Otherwise, the maximum velocity must be scaled down
        #print(f"k_v before condition = \n{k_v}")
        mask = (abs(D) <= (3/2 * k_v**2/k_a))
        k_v[mask] = np.sqrt(2/3 * abs(D[mask]) * k_a[mask])
        #print(f"k_v after condition = \n{k_v}")
        
        #Synchronizing each point for each leg
        k_v, self.h = self.synchronize_linear(k_v, D)
        
        #Synchronized segment time and velocities
        self.t, self.T, self.t_f, self.segment_velocity = self.set_time_and_velocity(k_v, k_a, self.h, D, sign_D)

        #Class wide flags
        self.printed = False
        self.done = False
        self.prev_segment_index = -1

    def synchronize_linear(self, k_v, D):
        """
        Synchronize segment velocities such that each joint in each leg have the same segment time. 
        This results in the joints of one leg reaching all it's points at the same time.
        
        We do not synchronize across legs, which means that one leg can finish it's trajectory before another.
        """  
        D = abs(D)
        
        #h is the time of each linear segment based on the distance of the segment and the velocity of the segment.
        #based on unsynchronized velocities
        # [13.59]
        mask = k_v != 0
        h = np.zeros(k_v.shape)
        h[mask] = D[mask]/k_v[mask]
        
        _lambda = np.zeros(k_v.shape)
        
        #print(f"k_v before sync = \n{k_v}")
        #print(f"D in sync : {D}")

        for i in range(self.num_joints // 2): #Do this for every leg. There are 2 joints in each leg
            for k in range(self.num_segments):
                #Using these indicies we fetch linear segment time, h, segment distance, D, and segment speed, k_v, for the current leg.
                start_index, end_index = (2 * i, 2 * (i + 1))

                h_leg = h[k,start_index:end_index]
                
                k_v_leg = k_v[k,start_index:end_index]
                D_leg = D[k,start_index:end_index]
                
                _lambda_leg = np.zeros(k_v_leg.shape)
                
                #We scale k_v_1 and k_v_2, so that they spend the same amount of time, h, in the current linear segment.
                #Assuming neither D_1 nor D_2 are equal to 0. If one of them are, the velocity remain unchanged. 
                #Setting _lambda = 0 means setting k_v = 0 resulting in h = 0 which again results in skipping the segment entirely. 
                if D_leg[0] == 0 and D_leg[1] == 0:
                    _lambda_leg = np.zeros(2)
                elif D_leg[0] == 0:
                    _lambda_leg[0] = 0
                    _lambda_leg[1] = 1
                elif D_leg[1] == 0:
                    _lambda_leg[0] = 1
                    _lambda_leg[1] = 0
                else:
                    #[13.34]
                    #We only scale velocities. This will only synchronize linear segments, not blend regions. 
                    _lambda_leg[0] = np.minimum(1, D_leg[0] * k_v_leg[1] / (D_leg[1] * k_v_leg[0])) #D = abs(D)
                    _lambda_leg[1] = np.minimum(1, D_leg[1] * k_v_leg[0] / (D_leg[0] * k_v_leg[1]))
                
                #Update k_v with the synchronized values for the current leg.
                k_v[k,start_index:end_index] = k_v[k,start_index:end_index] * _lambda_leg
        
        mask = k_v != 0
        #print(f"h before sync\n {h}")
        h[mask] = D[mask]/k_v[mask] #h is now based on syncronized velocities
        #print(f"h after sync\n {h}")
        #print(f"k_v after sync = \n{k_v}")

        return k_v, h
    
    def synchronize_initial_and_final_blend(self, k_v, k_a):
        
        T = np.zeros((2, 8))
        
        for i in range(self.num_joints // 2):
            start_index, end_index = (2 * i, 2 * (i + 1))
            
            for k in range(2): #Only for segment 0 and -1
                k = k - 1 
                
                _v = np.zeros(2)
                
                k_v_leg = k_v[k,start_index:end_index]
                
                if k_v_leg[1] == 0 and k_v_leg[0] == 0:
                    _v[0] = _v[1] = 0
                elif k_v_leg[1] == 0:
                    _v[0] = 1
                elif k_v_leg[0] == 0:
                    _v[1] = 1
                else:
                    _v[0] = np.minimum(1, k_v_leg[0] / k_v_leg[1])
                    _v[1] = np.minimum(1, k_v_leg[1] / k_v_leg[0])
                
                k_a_leg = k_a[k,start_index:end_index] * _v
                
                #print(f"k_a_leg {k_a_leg}")
                
                T_leg = T[k, start_index:end_index]
                
                #mask = k_a_leg != 0
                
                
                if k_a_leg[0] == 0:
                    T_leg[0] = 0
                else: 
                    T_leg[0] = 3/4 * k_v_leg[0]/k_a_leg[0]
                
                if k_a_leg[1] == 0:
                    T_leg[1] = 0
                else: 
                    T_leg[1] = 3/4 * k_v_leg[1]/k_a_leg[1]
        
        return T[0], T[-1]
            
    def set_time_and_velocity(self, k_v, k_a, h, D, sign_D):  
        #Locally store globals
        num_points = self.num_points
        num_joints = self.num_joints
        num_segments = self.num_segments
        
        segment_velocity = k_v * sign_D
        
        T = np.zeros((num_points, num_joints))
        t = np.zeros((num_points, num_joints))
        
        
        #Find the time of each blend acc. to:
        # [13.60]
        
        T[0], T[-1] = self.synchronize_initial_and_final_blend(k_v, k_a)
        
        #T[0] = 3/4 * abs(segment_velocity[0])/k_a
        #T[-1] = 3/4 * abs(segment_velocity[-1])/k_a
        
        for i in range(1, num_segments):
            T[i] = 3/4 * abs(segment_velocity[i] - segment_velocity[i - 1]) / k_a[0]
        
        # Make sure that : h_k >= T_k + T_k+1. If this is not upheld, the segment will end before we are done with the blend regions.
        # If this is not true, we must set h_k = T_k + T_k+1
        # Increasing h, means that we must also update segment_velocity, otherwise h_k * segment_velocity_k != D_k
        for k in range(num_segments):
            for j in range(num_joints):
                if h[k, j] < T[k, j] + T[k + 1, j]:
                    #print(f"Joint {j}, Segment {i}. H is smaller than Tk + Tk+1")
                    #print(f"{h[k, j]} < {T[k, j]} + {T[k + 1, j]}")
                    
                    h[k, j] = T[k, j] + T[k + 1, j]
                    segment_velocity[k, j] = D[k, j] / h[k, j]

                    #Find index of other joint in same leg and resync velocities
                    if j % 2 == 0:
                        j_other = j + 1
                    else:
                        j_other = j - 1

                    h[k, j_other] = h[k, j]
                    segment_velocity[k, j_other] = D[k, j_other] / h[k, j_other]


        # Finding the start time of each blend acc. to a modified version of:
        # [13.63]
        for k in range(1, num_points):
            if k == 1:
                t[1] = t[0] + T[0] + h[0] - T[1]
            else:   
                t[k] = t[k - 1] + T[k - 1] + h[k - 1] - T[k]
        
        # The end time of the entire trajectory is equal to the last blend start time + the time spent in the blend in question.
        t_f = t[-1] + 2*T[-1]
        
        #print(f"h = \n {h}")
        #print(f"T = \n {T}")

        return t, T, t_f, segment_velocity
        
    def via(self, t_c):
        #Locally store globals
        num_joints = self.num_joints
        num_segments = self.num_segments
        
        T = self.T
        t = self.t
        t_f = self.t_f
        
        segment_velocities = self.segment_velocity
        points = self.points
        
        #Initiate q and q_prime. These will eventually hold all the position and velocity references for the current time
        q = np.zeros(num_joints)
        q_prime = np.zeros(num_joints)
        
        #List of indicies corresponding to which segment each joint is currently traversing, this will be incremented continuously
        segment_index = self.segment_index
        
        #Do this for every joint
        for j in range(num_joints):
            #First, determine the joints current segment
            k = segment_index[j] 
            
            #Find the next_blend_start_time, which we use to check, whether the segment is finished
            if k < num_segments:
                next_blend_start_time = t[k + 1, j]
            else:
                #If we are at the last segment, t_k+1 does not exist. Set next_blend_start_time to the final time
                next_blend_start_time = t_f[j]
            
            #If the current time has passed next_blend_start_time. Increment the segment index. 
            #This has to be a while loop, in case two segments have the same end time. Happens if there are two equal points
            while t_c >= next_blend_start_time and next_blend_start_time < t_f[j]:
                k = k + 1
                self.segment_index[j] = k #Update stored index. k is local
                
                #When the segment index is incremented, we must also update next_blend_start_time
                if k < num_segments:
                    next_blend_start_time = t[k + 1, j]
                else:
                    #t_k+1 does not exist for the last segment. Same as above
                    next_blend_start_time = t_f[j]
            
            #Now that the correct segment and next_blend_start_time is determined, we can find the relevant values used in calculation.
            point = points[k, j]
            
            blend_time = T[k, j]  
            blend_start_time = t[k, j]
            blend_end_time = blend_start_time + 2 * blend_time
            
            #Segment velocity has two special cases. 
            #At the first segment : previous segment velocity = 0
            #At the last segment: current segment velocity = 0
            if k < num_segments: #Last segment
                segment_velocity = segment_velocities[k, j]
            else:
                segment_velocity = 0    
            
            if k == 0: #First segment
                prev_segment_velocity = 0
            else:
                prev_segment_velocity = segment_velocities[k - 1, j]
                
            delta_segment_velocity = segment_velocity - prev_segment_velocity
            
            #Blend region
            if t_c >= blend_start_time and t_c < blend_end_time: 
                #[13.65]
                q[j] = (
                    point 
                    - 1/(16*blend_time**3) * (t_c - blend_start_time)**3 * (t_c - blend_start_time - (4*blend_time)) * delta_segment_velocity 
                    + (t_c - blend_start_time - blend_time) * prev_segment_velocity
                )
                
                #[13.66]
                q_prime[j] = (
                    prev_segment_velocity 
                    - 1/(4 * blend_time**3) * (t_c - blend_start_time)**2 * (t_c - blend_start_time - 3*blend_time) * delta_segment_velocity
                )
            
            #Linear region
            elif t_c >= blend_end_time and t_c <= next_blend_start_time: # Linear segment # self.t[j, k+1] is the one term that makes us expand q with another q_f   
                #[13.62]
                q[j] = (
                    (t_c - blend_start_time - blend_time) * segment_velocity
                    + point
                )
                
                #The velocity is just the velocity of the segment
                q_prime[j] = segment_velocity
        
        return q, q_prime
        
    def step(self, t):
        #Locally store globals 
        t_f = self.t_f
        q_f = self.q_f       
        
        #Calculate current position and velocity based on the current time
        q, q_prime = self.via(t)
        
        #Check wether all the joints have exceeded their final time
        done = True
        for i in range(self.num_joints):
            if t >= t_f[i]:
                q[i] = q_f[i]
                q_prime[i] = 0
            else:
                done = False
        if done:
            self.done = done

        return q, q_prime

    def is_done(self):
        return self.done

    def is_valid(self):
        #Locally store globals
        joint_limits = config.joint_limits
        q_f = self.q_f
        
        valid = True
        for i in range(len(q_f)):
            if q_f[i] > joint_limits[0][i]:
                valid = False
            if q_f[i] < joint_limits[1][i]:
                valid = False
        return valid