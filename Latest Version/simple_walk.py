import numpy as np

def tilt(theta, coordinates):
    new_coordinates = np.zeros((4, 2))
    
    for i in range(len(coordinates)):
        coordinate = coordinates[i]
        
        x = coordinate[0]
        y = coordinate[1]
        
        y = 320 - y
        
        h = np.sqrt(x**2 + y**2)
    
        sigma = np.arctan(x/y)
        
        new_coordinates[i][0] = h * np.sin(theta + sigma)
        y = h * np.cos(theta + sigma)
        
        new_coordinates[i][1] = 320 - y

    return new_coordinates




#Defining tilt amount
l = 390

theta = np.radians(13)
delta_y = l * np.sin(theta)

#### Defining gait parameters

stride_length_front = 75
stride_length_hind = 100

kick_amount = 20

back_leg_center = -60

lift_amount = 80


home = np.array([
    [0,100 + delta_y / 2],
    [0,100 + delta_y / 2],

    [back_leg_center,100 - delta_y / 2],
    [back_leg_center,100 - delta_y / 2]
])

left_leg_back = home + np.array([
    [-stride_length_front/2, 0],
    [stride_length_front/2, lift_amount/4], # + less_up
    
    [stride_length_hind/2, lift_amount/4], # + 20 , +20
    [-stride_length_hind/2, -kick_amount]
])

left_leg_up = home + np.array([
    [-stride_length_front/2, lift_amount],
    [0, 0], 

    [0, 0], 
    [-stride_length_hind/2, lift_amount]
])

left_leg_forward = home + np.array([
    [stride_length_front/2, lift_amount/4], 
    [-stride_length_front/2 , 0],
    
    [-stride_length_hind/2 , -kick_amount],
    [stride_length_hind/2, lift_amount/4] 
])

left_leg_down = home + np.array([
    [0,0], 
    [-stride_length_front/2, lift_amount],
    
    [-stride_length_hind/2, lift_amount],
    [0,0] 
])

back_to_forward_velocity = np.array([
    15.0,15.0,
    1,1,
    1,1,
    15.0,15.0
])

back_to_forward_velocity = back_to_forward_velocity * 1

forward_to_back_velocity = np.array([
    1,1,
    15.0,15.0,
    15.0,15.0,
    1,1
])

forward_to_back_velocity * 1

acceleration = np.full(8, 100.0)


left_leg_back = tilt(theta, left_leg_back)
left_leg_up = tilt(theta, left_leg_up)
left_leg_forward = tilt(theta, left_leg_forward)
left_leg_down = tilt(theta, left_leg_down)


'''
#Right leg forward
left_leg_back = np.array([
    #Front
    [-forward_distance * 2, height + leaning_offset], #Left
    [forward_distance, height + leaning_offset + less_up], #Right
    #Hind
    [forward_distance + 20, height + 20],
    [-forward_distance * 2, height]
])

#Right leg down
left_leg_up = np.array([
    #Front
    [-forward_distance, height * 2 + leaning_offset], #Left
    [mid_point, height - down_distance - 10 + leaning_offset], #Right
    #Hind
    [mid_point, height - down_distance], #Left
    [-forward_distance, height * 2] #Right
])

#Right leg back
left_leg_forward = np.array([
    #Front
    [forward_distance, height + leaning_offset + less_up], #Left
    [-forward_distance * 2, height + leaning_offset], #Right
    #Hind
    [-forward_distance * 2, height], #Left
    [forward_distance + 20, height + 20] #Right
])

#Right leg up
left_leg_down = np.array([
    #Front
    [mid_point, height - down_distance - 10 + leaning_offset], #Left
    [-forward_distance, height * 2 + leaning_offset], #Right
    #Hind
    [-forward_distance, height * 2], #Left
    [mid_point, height - down_distance] #Right
])
'''

#siste endringer:
#Fart 0.5 -> 1
#[forward_distance + 20, height + 40] -> [forward_distance + 20, height + 20]

