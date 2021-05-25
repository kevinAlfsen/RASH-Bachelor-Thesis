import numpy as np

dt = 0.001

N_SLAVES_CONTROLLED = 4
N_SLAVES = 4

num_joints = N_SLAVES * 2

gear_ratio = 9

current_saturation = 5

joint_limits = [[3*np.pi for i in range(num_joints)],[-3*np.pi for i in range(num_joints)]]

kp = 5
kd = 0.03

default_speed = np.full(8, 5.0) #max velocity per joint, output space, rad/s
default_acceleration = np.full(8, 10.0) #max acceleration per joint, output space, rad/s**2

down_speed = np.full(8, 10)
down_acceleration = np.full(8, 20)

stand_config = np.array([np.pi/4, -np.pi/2, -np.pi/4, np.pi/2, -np.pi/4, np.pi/2, np.pi/4, -np.pi/2])
home_config = np.zeros(8)


walk_mode_home = np.array([0,0,0,0, np.pi, 0, -np.pi, 0])
idle_mode_home = np.zeros(8)


home_coordinates = np.array([
    [0, 320],
    [0, 320],
    [0, 320],
    [0, 320]
])

