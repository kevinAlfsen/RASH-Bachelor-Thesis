import numpy as np

#go to idle
#go to 

#loop these coordinates

lean_amount = 0
back_leg_offset = 0


tilt = np.radians(20)


x = 0
y = 100

l = 380

new_x = np.cos(tilt) * x
new_y = np.cos(tilt) * y

delta_y = l * np.sin(tilt)
delta_y_array = np.array([
    [0, delta_y/2],
    [0, delta_y/2],
    [0, -delta_y/2],
    [0, -delta_y/2]
], dtype=float)


home = np.array([
    #Front
    [0, 100 + lean_amount], #Left
    [0, 100 + lean_amount], #Right
    #Hind
    [0 - back_leg_offset, 100 - lean_amount], #Left
    [0 - back_leg_offset, 100 - lean_amount]  #Right
], dtype = float)


lift_amount = 30

#Right leg down
_front_left_up = home + np.array([
    #Front
    [0, lift_amount], #Left
    [0,lift_amount], #Right
    #Hind
    [0,0], #Left
    [0,0]  #Right
], dtype = float)

#Right leg down
_front_right_up = home + np.array([
    #Front
    [0,0], #Left
    [0,0], #Right
    #Hind
    [-40,50], #Left
    [-40,50]  #Right
], dtype = float)

front_left_up = np.cos(tilt) * _front_left_up + delta_y_array #tilt(_front_left_up)
front_right_up = np.cos(tilt) * _front_right_up + delta_y_array #tilt(_front_right_up)

velocity_idle = np.array([
    2.0,2.0,
    2.0,2.0,
    4.0,4.0,
    4.0,4.0
])

acceleration = np.full(8, 200.0)

velocity = np.array([
    2.0,2.0,
    2.0,2.0,
    4.0,4.0,
    4.0,4.0
])


#def tilt(coordinates):
#    new_coordinates = np.array((4, 2), dtype=float)

#    for i in range(len(coordinates)):
#        new_coordinates[i] = coordinates[i] * np.cos(tilt)

#siste endringer:
#Fart 0.5 -> 1
#[forward_distance + 20, height + 40] -> [forward_distance + 20, height + 20]

