import numpy as np
import config

class Trajectory:
    def __init__(self, q_f, delay=0):
        self.delay = delay

        k_v = config.k_v
        k_a = config.k_a

        self.k_v = np.array(k_v)
        self.k_a = np.array(k_a)


        self.q_f = np.array(q_f)

        self.num_joints = len(q_f)
        self.done = False

        #self.t_prev = None
   

    def sign(self, x):
            if (x == 0):
                return 0
            else:
                return x/abs(x)

    def set_initial_conditions(self, q_i, t_i):
        self.q_i = np.array(q_i)
        self.t_i = t_i

        self.D = self.q_f - self.q_i

        self.sign_D = np.zeros(8)

        for i in range(len(self.D)):
            self.sign_D[i] = self.sign(self.D[i])

        #v_sign = np.vectorize(sign, otypes=[np.float])

        #self.sign_D = v_sign(self.D)
        
        self.k_v, self.k_a, self.tau, self.t_f = self.synchronize(self.k_v, self.k_a)

    def synchronize(self, k_v, k_a):

        mask = (abs(self.D) <= k_v**2/k_a) #This being true, means that the joint will not be able to achieve its max velocity. 
        k_v[mask] = np.sqrt(abs(self.D[mask]) * k_a[mask]) #With these elements, we set k_v to the maximum achievable velocity

        tau = k_v / k_a

        travel_time = np.zeros(self.num_joints)

        mask = self.k_v != 0 #The result of correcting the k_v elements, is that some of them are 0, if D is 0
        travel_time[mask] = tau[mask] + abs(self.D[mask])/k_v[mask] #The elements where k_v is not 0 is calculated normally, the others are left at 0 travel time

        #print(f"Travel_time : {travel_time}\n\n")

        max_travel_time = max(travel_time)
        max_travel_time_index = np.where(travel_time == max_travel_time)[0][0]

        tau_prime = tau[max_travel_time_index]

        _lambda = np.zeros(self.num_joints)
        _v = np.zeros(self.num_joints)

        mask = self.k_v != 0 #Skipping the cases where D == 0¸ these elements will stay 0

        _lambda[mask] = k_v[max_travel_time_index]*abs(self.D[mask])/(k_v[mask]*abs(self.D[max_travel_time_index]))
        _v[mask] = k_a[max_travel_time_index]*abs(self.D[mask])/(k_a[mask]*abs(self.D[max_travel_time_index]))

        _lambda = np.minimum(1, _lambda)
        _v = np.minimum(1, _v)

        k_v = _lambda * k_v
        k_a = _v * k_a

        #print(f"k_v : {np.round(k_v,3)}\nk_a : {np.round(k_a, 3)}\ntau_prime : {np.round(tau_prime, 3)}\nmax_travel_time : {np.round(max_travel_time, 3)}\n\n")
        #print(f"Q_final : {np.degrees(self.q_f)}\nMax Travel Time : {max_travel_time}\n")

        return k_v, k_a, tau_prime, max_travel_time
        
    def trapeze(self, t_c):
        """
        Trapeze velocity profile,
        this is the fastest of the proposed joint trajectories given that |D_j| > k_v^2/k_a.
        It is also possible to sync up with and get the same travel time as other trajectories of the same type
        """
        q = np.zeros(self.num_joints)
        q_prime = np.zeros(self.num_joints)
        #q_prime_prime = np.zeros(self.num_joints)
        #q_integral = np.zeros(self.num_joints)
        
        if t_c <= self.tau:
            q = self.q_i + 1/2 * t_c**2 * self.k_a * self.sign_D
            q_prime = t_c * self.k_a * self.sign_D
            #if t_prev != None:
            #    q_integral = (self.q_i * t_c + 1/6 * self.k_a * self.sign_D * t_c**3) - (self.q_i * t_prev + 1/6 * self.k_a * self.sign_D * t_prev**3)
        elif t_c >= self.tau and t_c <= (self.t_f - self.tau):
            q = self.q_i + (t_c - self.tau/2) * self.k_v * self.sign_D
            q_prime = self.k_v * self.sign_D 
            #if t_prev != None:
            #    q_integral = (self.q_i * t_c + self.k_a * self.sign_D * (1/2 * t_c**2 - self.tau/2 * t_c)) - (self.q_i * t_prev + self.k_a * self.sign_D * (1/2 * t_prev**2 - self.tau/2 * t_prev))
        elif t_c >= (self.t_f - self.tau) and t_c <= self.t_f:
            q = self.q_f - 1/2 * (self.t_f - t_c)**2 * self.k_a * self.sign_D
            q_prime = -self.k_a * (t_c - self.t_f) * self.sign_D
            #if t_prev != None:
            #    q_integral = (self.q_f * t_c - 1/2 * self.k_a * self.sign_D * (self.t_f**2 * t_c - self.t_f * t_c**2 + 1/3 * t_c**3)) - (self.q_f * t_prev - 1/2 * self.k_a * self.sign_D * (self.t_f**2 * t_prev - self.t_f * t_prev**2 + 1/3 * t_prev**3))
        elif t_c > self.t_f:
            q = self.q_f

        #self.t_prev = t_c

        return q, q_prime #, q_integral
    
    def Done(self):
        return self.done

    def step(self, t):
        t_c = t - self.t_i

        '''
        if t_c > self.t_f:
            #Start counting down delay
            if t_c - self.t_f >= self.delay:
                self.done = True

            #self.done = True
            #q, q_prime = self.trapeze(self.t_f)
            return self.q_f, np.zeros(self.num_joints)

        else:

            q, q_prime = self.trapeze(t_c)
            return q, q_prime
        '''


        q, q_prime = self.trapeze(t_c)
        
        self.done = True
        for i in range(len(q)):
            if (abs(self.q_f[i] - q[i])) > 0:
                self.done = False

        return q, q_prime

    def is_valid(self):
        valid = True

        for i in range(len(self.q_f)):
            if self.q_f[i] > config.joint_limits[0][i]:
                valid = False
            if self.q_f[i] < config.joint_limits[1][i]:
                valid = False

        return valid
