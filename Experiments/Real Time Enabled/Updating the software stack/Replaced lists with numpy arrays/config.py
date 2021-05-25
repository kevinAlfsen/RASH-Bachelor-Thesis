import numpy as np

dt = 0.001

N_SLAVES_CONTROLLED = 4
N_SLAVES = 4

num_joints = N_SLAVES * 2

gear_ratio = 9

current_saturation = 1

joint_limits = [[3*np.pi for i in range(num_joints)],[-3*np.pi for i in range(num_joints)]]

kp = 5
kd = 0.03

k_v = [20 for i in range(8)] #max velocity per joint, output space, rad/s
k_a = [80 for i in range(8)] #max acceleration per joint, output space, rad/s**2