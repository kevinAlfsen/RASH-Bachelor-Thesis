import numpy as np

dt = 0.001

N_SLAVES_CONTROLLED = 4
N_SLAVES = 4

num_joints = N_SLAVES * 2

gear_ratio = 9

current_saturation = 6

joint_limits = [[3*np.pi for i in range(num_joints)],[-3*np.pi for i in range(num_joints)]]

kp = 3    #4
kd = 0.04 #0.04


#Velocity and acelleration configurations
default_speed = np.full(8, 5.0) #max velocity per joint, output space, rad/s #2.5
default_acceleration = np.full(8, 50.0) #max acceleration per joint, output space, rad/s**2 #100

down_speed = np.full(8, 2.5)
down_acceleration = np.full(8, 10.0)

up_speed = np.full(8, 10.0)
up_acceleration = np.full(8, 40.0)


walk_home = np.array([0,0,0,0, np.pi, 0, -np.pi, 0])
idle_home = np.zeros(8)

idle_stand = np.array([np.pi/4, -np.pi/2, -np.pi/4, np.pi/2, -np.pi/4, np.pi/2, np.pi/4, -np.pi/2])
walk_stand = np.array([
    np.pi/4, -np.pi/2, 
    -np.pi/4, np.pi/2, 
    np.pi + np.pi/4, -np.pi/2, 
    -np.pi - np.pi/4, np.pi/2
])


home_coordinates = np.array([
    [0, 320],
    [0, 320],
    [0, 320],
    [0, 320]
])

