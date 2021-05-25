import numpy as np

#Kinematics parameters

l_1 = 160
l_2 = 160

hip_T_world = np.array([
    [-1,0,0],
    [0,1,-(l_1 + l_2)],
    [0,0,1]
])

world_T_hip = np.array([
    [-1,0,0],
    [0,1,l_1 + l_2],
    [0,0,1]
])

walk_zero_config = np.array([
    [0, -np.pi],
    [0, -np.pi],
    [np.pi, -np.pi],
    [np.pi, -np.pi]
])

idle_zero_config = np.array([
    [0, -np.pi],
    [0, -np.pi],
    [np.pi, -np.pi],
    [np.pi, -np.pi]
])

walk_joint_polarities = np.array([
    [-1,-1],
    [1,1],
    [-1,-1],
    [1,1]
])

idle_joint_polarities = np.array([
    [-1,-1],
    [1,1],
    [-1,-1],
    [1,1]
])

walk_elbow = [False, False, False, False]
idle_elbow = [False, False, True, True]

def simple_walk_controller(coordinates, current_position, zero_config=walk_zero_config, joint_polarities=walk_joint_polarities, elbows=walk_elbow):
    if coordinates.ndim > 2:
        num_points = len(coordinates)
        points = np.zeros((num_points, 8))
    else:
        num_points = 1
        points = np.zeros(8)

    for point_index in range(num_points):
        if num_points == 1:
            coordinate = coordinates
        else:
            coordinate = coordinates[point_index]

        q = np.zeros(8)

        if len(coordinate) != 4:
            print("Wrong Input, coordinates should be an array of 4, 2 array of x,y coordinates")
            return

        for i in range(len(coordinate)):

            #x = coordinates[i][0]
            #y = coordinates[i][1]
            #print(f"hip coordinates: {x,y}[mm]")

            p_foot = np.append(coordinate[i], 1)
            p_hip = hip_T_world.dot(p_foot) 
            
            x = p_hip[0]
            y = p_hip[1]

            #print(f"zero_config: {zero_config}")
            #print(f"joint_polarities: {joint_polarities}")

            #x = p_world[0]
            #y = p_world[1]
            
            if not is_within_reach(x, y):
                return [None]

            q_1, q_2 = inverse_kinematics_one_leg(x, y, elbows[i])
            
            #print(f"Inverse kinematics of : {x, y} gives : {np.degrees(q_1), np.degrees(q_2)}")

            q_1 = q_1 + zero_config[i][0]
            q_2 = q_2 + zero_config[i][1]

            q_1 = q_1 * joint_polarities[i][0]
            q_2 = q_2 * joint_polarities[i][1]

            #print("\nBefore shortest path:")
            #print(f"Initial : q_1 = {np.degrees(current_position[2*i])}, q_2 = {np.degrees(current_position[2*i + 1])}")
            #print(f"Final : q_1 = {np.degrees(q_1)}, q_2 = {np.degrees(q_2)}\n")

            q_1 = shortest_path(current_position[2*i], q_1)
            #q_2 = shortest_path(current_position[2*i + 1], q_2)

            #q_2 = current_position[2*i+1] + shortest_path(current_position[2*i+1], q_2)

            #q_2 = neg(q_2)

            #print("\nAfter shortest path:")
            #print(f"Final : q_1 = {np.degrees(q_1)}, q_2 = {np.degrees(q_2)}\n")



            q[2*i] = q_1
            q[2*i+1] = q_2

            if num_points == 1:
                points = q
            else:
                points[point_index] = q

    return points

def inverse_kinematics_one_leg(x,y, elbow_left = True):
    #print(f"elbow_left:{elbow_left}")
    r = np.sqrt(abs(x**2)+abs(y**2)) 

    gamma = np.arctan2(y,x)
    alpha = np.arccos((l_1**2 + l_2**2 - r**2) / (2*l_1*l_2))
    if r == 0:
        beta = 0
    else:
        beta = np.arccos((r**2+l_1**2-l_2**2) / (2*r*l_1))
    
    if elbow_left:
        q_2 = np.pi - alpha
        q_1 = gamma - beta
    else:
        q_2 = np.pi + alpha
        #print(f"q_2 = pi + {np.degrees(alpha)}")
        q_1 = gamma + beta

    return q_1, q_2


def forward_kinematics(q, end_effector_index=1):
    if len(q) != 8:
        print("Wrong input. q should be an array with 8 joint angles")
        return

    coordinates = []

    print(q)

    for i in range(len(q) // 2):
        coordinates.append(forward_kinematics_one_leg(q[i], q[i+1], end_effector_index))

    return coordinates

def forward_kinematics_one_leg(q_1, q_2, end_effector_index=1):
    
    q_1 = q_1 + config.zero_config_rotation[0]
    q_2 = q_2 + config.zero_config_rotation[1]
    
    x = np.zeros(config.num_joints // 4)
    y = np.zeros(config.num_joints // 4)


    x[0] = Config.l_1 * np.cos(q_1)
    y[0] = Config.l_1 * np.sin(q_1)
    
    x[1] = x[0] + Config.l_2 * np.cos(q_1 + q_2)
    y[1] = y[0] + Config.l_2 * np.sin(q_1 + q_2)

    p_hip = np.array([ x[end_effector_index], y[end_effector_index], 0, 1 ])
    
    p_world = Config.world_T_hip.dot(p_hip)
    
    return p_world[0], p_world[1]


def is_within_reach(x, y):
    if x**2 + y**2 > (l_1 + l_2)**2:
        print(f"({x},{y}) is not within reach")
        return False
    return True

def pos(angle):
    if angle < 0:
        return 2*np.pi + angle
    else:
        return angle

def neg(angle):
    if angle >= 0:
        return - 2 * np.pi + angle
    else:
        return angle 

def shortest_path(initial, final):
    distance_to_positive = pos(final) - initial
    distance_to_negative = abs(neg(final)) + initial

    if distance_to_positive < distance_to_negative:
        return pos(final)
    else:
        return neg(final)