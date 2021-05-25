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

lift = 40

home = np.array([
[0,100 + lift],
[0,100 - lift/2],
[-60 + 80,100],
[-60,100]
])

l = 390

theta = np.radians(10) #Max 30 deg
delta_y = l * np.sin(theta)

'''
coordinates = home + np.array([
    #x,y
    [0,0 + delta_y / 2], #FL
    [0,0 + delta_y / 2], #FR
    
    [-40,0 - delta_y / 2], #HL
    [-40,0 - delta_y / 2]  #HR
])
'''

coordinates = np.array([
    [
        [-50,50],
        [-50,50],
        [-200,200],
        [-200,200]
    ],
    [
        [0,0],
        [0,0],
        [0,0],
        [0,0]
    ]
])


#coordinates = tilt(theta, coordinates)

print(coordinates)

velocity = [
    #front left
    2, #hip
    2, #knee
    #front right
    2, #hip
    2, #knee
    #hind left
    2, #hip
    2, #knee
    #hind right
    2, #hip
    2 #knee
]


acceleration = [
    #front left
    10, #hip
    10, #knee
    #front right
    10, #hip
    10, #knee
    #hind left
    10, #hip
    10, #knee
    #hind right
    10, #hip
    10 #knee
]
