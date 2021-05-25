import numpy as np


#####################
# RAPPORT
####################

stride_length_right = 60        #80
stride_length_left = 60         #80

lift_amount = 55                #55
kick_amount = lift_amount/4

home = np.array([
    #Front
    [0, 100], #Left
    [0, 100],
    #Hind
    [0, 100], #Left 
    [0, 100]  #Right
], dtype = float)

# Lift HL and FR leg
HL_and_FR_up = home + np.array([
    [0 ,0], #FL

    [0, lift_amount], #FR
    [0 ,lift_amount], #HL

    [0,0] #HR
])

# Forwards
HL_and_FR_forward = home + np.array([
    [-stride_length_left/2,0], #FL

    [stride_length_right/2, 0], #FR
    [stride_length_left/2, 0], #HL

    [-stride_length_right/2,-kick_amount] #HR
])

HL_FR = np.array((HL_and_FR_up, HL_and_FR_forward), dtype=float)

# Lift HR and FL leg
HR_and_FL_up = home + np.array([
    [0 , lift_amount], #FL

    [0 , 0], #FR
    [0  , 0], #HL

    [0 , lift_amount] #HR
])

# Forwards
HR_and_FL_forward = home + np.array([
    [stride_length_left/2, 0], #FL

    [-stride_length_right/2, 0], #FR
    [-stride_length_left/2,-kick_amount], #HL

    [stride_length_right/2, 0] #HR
])

HR_FL = np.array((HR_and_FL_up, HR_and_FL_forward), dtype=float)

velocity = np.full(8, 2.0)


backward_motion = 1.5 
forward_motion = 5.5

HL_FR_velocity = np.array([
    backward_motion,backward_motion,
    forward_motion,forward_motion,
    forward_motion,forward_motion,
    backward_motion,backward_motion
], dtype=float)

HL_FR_velocity = HL_FR_velocity * 1

HR_FL_velocity = np.array([
    forward_motion,forward_motion,
    backward_motion,backward_motion,
    backward_motion,backward_motion,
    forward_motion,forward_motion
], dtype=float)

HR_FL_velocity = HR_FL_velocity * 1 #6

acceleration = np.full(8,100.0)

print(f"HL_and_FR_up = {HL_and_FR_up}")
print(f"HL_and_FR_forward = {HL_and_FR_forward}")

print(f"HR_and_FL_up = {HR_and_FL_up}")
print(f"HR_and_FL_forward {HR_and_FL_forward}")