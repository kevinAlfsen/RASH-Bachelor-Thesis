import numpy as np

class Config:
    # PD-settings from ZN-method
    # kp = 4
    # Kd = 0.0435

    kp = 5
    kd = 0.05

    current_saturation = 1

    interface_name = "enp4s0"

    N_SLAVES_CONTROLLED = 1
    N_SLAVES = 4

    num_joints = N_SLAVES_CONTROLLED * 2

    dt = 0.001

    joint_limits = [[3*np.pi for i in range(N_SLAVES * 2)],[-3*np.pi for i in range(N_SLAVES * 2)]]

    zero_calibration =np.array([0.026432,0.0,0.0,0.0,0.0,0.0,0.0,0.0]) #np.array([-0.01950692944,0,0,0,0,0,0,0])

    k_v = [10 for i in range(8)] #max velocity per joint, output space, rad/s
    k_a = [40 for i in range(8)] #max acceleration per joint, output space, rad/s**2

    gear_ratio = 9

    calibration_points = [
    0, 
    np.pi/4, 2*np.pi/4, 3*np.pi/4, 4*np.pi/4, 5*np.pi/4, 6*np.pi/4, 7*np.pi/4, 
    8*np.pi/4, 
    7*np.pi/4, 6*np.pi/4, 5*np.pi/4, 4*np.pi/4, 3*np.pi/4, 2*np.pi/4, np.pi/4,
    0
    ]

    #calibration_points = [4*np.pi - i*np.pi/4 for i in range(17)]

    calibration_delay = 4

def Calibration_parameters(motor_index):
    for i in range(len(Config.calibration_points)):
        cal_params = [0,0,0,0,0,0,0,0]
        cal_params[motor_index] = Config.calibration_points[i]
        yield cal_params
