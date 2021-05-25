import numpy as np
'''
########################################
#Daniel Notes:
########################################
stride_length_left = 50
stride_length_right = 50
#stride_length_hind = stride_length * 2

lift_amount = 50
lift_amount_front = lift_amount
kick_amount = lift_amount/6

home = np.array([
    #Front
    [0, 100], #Left
    [0, 100],
    #Hind
    [-10, 100], #Left
    [-10, 100]  #Right
], dtype = float)


HL_and_FR_up = home + np.array([
    [stride_length_left/2,0], #FL
    [stride_length_right/2,lift_amount_front], #FR
    [stride_length_left/2,lift_amount], #HL
    [stride_length_right/2,0] #HR
])

HL_and_FR_forward = home + np.array([
    [0,0],
    [stride_length_right, 0],
    [stride_length_left, 10],
    [0,-kick_amount]
])

HL_FR = np.array((HL_and_FR_up, HL_and_FR_forward), dtype=float)

HR_and_FL_up = home + np.array([
    [stride_length_left/2, lift_amount_front], #FL
    [stride_length_right/2, 0], #FR
    [stride_length_left/2, 0], #HL
    [stride_length_right / 2, lift_amount] #HR
])

HR_and_FL_forward = home + np.array([
    [stride_length_left, 0],
    [0, 0],
    [0,-kick_amount],
    [stride_length_right, 10]
])

HR_FL = np.array((HR_and_FL_up, HR_and_FL_forward), dtype=float)

velocity = np.full(8, 2.0)

HL_FR_velocity = np.array([
    1.5,1.5,
    5.5,5.5,
    5.5,5.5,
    1.5,1.5
], dtype=float)

HL_FR_velocity = HL_FR_velocity * 4

HR_FL_velocity = np.array([
    5.5,5.5,
    1.5,1.5,
    1.5,1.5,
    5.5,5.5
], dtype=float)

HR_FL_velocity = HR_FL_velocity * 4

acceleration = np.full(8,200.0)
'''


'''
########################################
# More height and "speed", less stable
########################################
stride_length = 45

lift_amount = 60
kick_amount = lift_amount/4

home = np.array([
    #Front
    [0, 70], #Left
    [0, 70],
    #Hind
    [-10, 70], #Left
    [-10, 70]  #Right
], dtype = float)


HL_and_FR_up = home + np.array([
    [stride_length/2,0], #FL
    [0,lift_amount], #FR
    [0,lift_amount], #HL
    [stride_length/2,0] #HR
])

HL_and_FR_forward = home + np.array([
    [0,0],
    [stride_length, 0],
    [stride_length, 0],
    [0,-kick_amount]
])

HL_FR = np.array((HL_and_FR_up, HL_and_FR_forward), dtype=float)

HR_and_FL_up = home + np.array([
    [0, lift_amount], #FL
    [stride_length/2, 0], #FR
    [stride_length/2, 0], #HL
    [0, lift_amount] #HR
])

HR_and_FL_forward = home + np.array([
    [stride_length, 0],
    [0, 0],
    [0,-kick_amount],
    [stride_length, 0]
])

HR_FL = np.array((HR_and_FL_up, HR_and_FL_forward), dtype=float)

velocity = np.full(8, 2.0)


backward_motion=1.5 
forward_motion = 5.5

HL_FR_velocity = np.array([
    backward_motion,backward_motion,
    forward_motion,forward_motion,
    forward_motion,forward_motion,
    backward_motion,backward_motion
], dtype=float)

HL_FR_velocity = HL_FR_velocity * 4

HR_FL_velocity = np.array([
    forward_motion,forward_motion,
    backward_motion,backward_motion,
    backward_motion,backward_motion,
    forward_motion,forward_motion
], dtype=float)

HR_FL_velocity = HR_FL_velocity * 4

acceleration = np.full(8,200.0)
'''



#################################################
#Stable small lift amount. To be used in live demos
##################################################


'''
direction_right = np.array([
    [0,0],
    [0,0],
    [0,0],
    [0,0]
])

direction_left = np.array([
    [0,0],
    [0,0],
    [0,0],
    [0,0]
])

direction_backwards = np.array([
    [0,0],
    [0,0],
    [0,0],
    [0,0]
])

direction_forwards = np.array([
    [0,0],
    [0,0],
    [0,0],
    [0,-1]
])

direction = direction_backwards
'''
#stride_length_hind = stride_length * 2


'''
#Mye stride

stride_length_right = 50
stride_length_left = 50

lift_amount = 50
lift_amount_front = lift_amount * 1
kick_amount = lift_amount/6
damping = 10

home = np.array([
    #Front
    [0, 100], #Left
    [0, 100],
    #Hind
    [-stride_length_right/2, 100], #Left
    [-stride_length_right/2, 100]  #Right
], dtype = float)

# Lift HL and FR leg
HL_and_FR_up = home + np.array([
    [stride_length_left/2 ,0], #FL

    [stride_length_right/2,lift_amount_front], #FR
    [stride_length_left/2 ,lift_amount], #HL

    [stride_length_right/2,0] #HR
])

# Forwards
HL_and_FR_forward = home + np.array([
    [0,0], #FL

    [stride_length_right, 0], #FR
    [stride_length_left, damping], #HL

    [0,-kick_amount] #HR
])

HL_FR = np.array((HL_and_FR_up, HL_and_FR_forward), dtype=float)

# Lift HR and FL leg
HR_and_FL_up = home + np.array([
    [stride_length_left/2  , lift_amount_front], #FL

    [stride_length_right/2 , 0], #FR
    [stride_length_left/2  , 0], #HL

    [stride_length_right/2 , lift_amount] #HR
])

# Forwards
HR_and_FL_forward = home + np.array([
    [stride_length_left, 0], #FL

    [0, 0], #FR
    [0,-kick_amount], #HL

    [stride_length_right, damping] #HR
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

HL_FR_velocity = HL_FR_velocity * 3

HR_FL_velocity = np.array([
    forward_motion,forward_motion,
    backward_motion,backward_motion,
    backward_motion,backward_motion,
    forward_motion,forward_motion
], dtype=float)

HR_FL_velocity = HR_FL_velocity * 3 #6

acceleration = np.full(8,250.0)
'''


#####################3
# RASK TROT
####################
stride_length_right = 75
stride_length_left = 75

lift_amount = 55
lift_amount_front = lift_amount * 1
kick_amount = lift_amount/4
damping = 10

home = np.array([
    #Front
    [0, 100], #Left
    [0, 100],
    #Hind
    [-stride_length_right/2, 100], #Left
    [-stride_length_right/2, 100]  #Right
], dtype = float)

# Lift HL and FR leg
HL_and_FR_up = home + np.array([
    [stride_length_left/2 ,0], #FL

    [stride_length_right/2,lift_amount_front], #FR
    [stride_length_left/2 ,lift_amount], #HL

    [stride_length_right/2,0] #HR
])

# Forwards
HL_and_FR_forward = home + np.array([
    [0,0], #FL

    [stride_length_right, 0], #FR
    [stride_length_left, damping], #HL

    [0,-kick_amount] #HR
])

HL_FR = np.array((HL_and_FR_up, HL_and_FR_forward), dtype=float)

# Lift HR and FL leg
HR_and_FL_up = home + np.array([
    [stride_length_left/2  , lift_amount_front], #FL

    [stride_length_right/2 , 0], #FR
    [stride_length_left/2  , 0], #HL

    [stride_length_right/2 , lift_amount] #HR
])

# Forwards
HR_and_FL_forward = home + np.array([
    [stride_length_left, 0], #FL

    [0, 0], #FR
    [0,-kick_amount], #HL

    [stride_length_right, damping] #HR
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

HL_FR_velocity = HL_FR_velocity * 7

HR_FL_velocity = np.array([
    forward_motion,forward_motion,
    backward_motion,backward_motion,
    backward_motion,backward_motion,
    forward_motion,forward_motion
], dtype=float)

HR_FL_velocity = HR_FL_velocity * 7 #6

acceleration = np.full(8,350.0)

'''
######################################
#KEVIN STARTET HER
######################################

HL_and_FR_up = home + np.array([
    [stride_length_left/2 ,0], #FL
    [stride_length_right/2,lift_amount_front], #FR
    [stride_length_left/2 ,lift_amount], #HL
    [stride_length_right/2,0] #HR
])

HL_and_FR_forward = home + np.array([
    [0,0],
    [stride_length_right, 10],
    [stride_length_left, 0],
    [0,0]
]) 

HL_FR = np.array((HL_and_FR_up, HL_and_FR_forward), dtype=float)

HR_and_FL_up = home + np.array([
    [stride_length_left/2  , lift_amount_front], #FL
    [stride_length_right/2 , 0], #FR
    [stride_length_left/2  , 0], #HL
    [stride_length_right/2 , lift_amount] #HR
]) + (kick_amount * direction)

HR_and_FL_forward = home + np.array([
    [stride_length_left, 10],
    [0, kick_amount],
    [0,0],
    [stride_length_right, 0]
])

#####################################
# Kevin sluttet her
#####################################
'''



#Hører til mye stride




print(f"HL_and_FR_up = {HL_and_FR_up}")
print(f"HL_and_FR_forward = {HL_and_FR_forward}")

print(f"HR_and_FL_up = {HR_and_FL_up}")
print(f"HR_and_FL_forward {HR_and_FL_forward}")