import numpy as np

home = np.array([
    [0,50],
    [0,50],
    [0,50],
    [0,50]
])


descent = 40
forward = 50

HL_lowering = home + np.array([
    #x,y
    [0,descent/2], #FL
    [0,descent], #FR
    
    [0,0], #HL
    [0,descent/2]  #HR
])

HL_lifting = home + np.array([
    #x,y
    [0,descent/2], #FL
    [0,descent], #FR
    
    [forward,descent/2], #HL
    [0,descent/2]  #HR
])

HL_home = home + np.array([
    #x,y
    [0,0], #FL
    [0,0], #FR
    
    [forward,0], #HL
    [0,0]  #HR
])

HR_lowering = home + np.array([
    #x,y
    [0,descent], #FL
    [0,descent/2], #FR
    
    [forward,descent/2], #HL
    [0,0]  #HR
])

HR_lifting = home + np.array([
    #x,y
    [0,descent], #FL
    [0,descent/2], #FR
    
    [forward,descent/2], #HL
    [forward,descent/2]  #HR
])

HR_home = home + np.array([
    #x,y
    [0,0], #FL
    [0,0], #FR
    
    [forward,0], #HL
    [forward,0]  #HR
])

FR_lowering = home + np.array([
    #x,y
    [0,descent/2], #FL
    [0,0], #FR
    
    [forward,descent], #HL
    [forward,descent/2]  #HR
])

FR_lifting = home + np.array([
    #x,y
    [0,descent/2], #FL
    [forward,descent/2], #FR
    
    [forward,descent], #HL
    [forward,descent/2]  #HR
])

FR_home = home + np.array([
    #x,y
    [0,0], #FL
    [forward,0], #FR
    
    [forward,0], #HL
    [forward,0]  #HR
])

FL_lowering = home + np.array([
    #x,y
    [0,0], #FL
    [forward,descent/2], #FR
    
    [forward,descent/2], #HL
    [forward,descent]  #HR
])

FL_lifting = home + np.array([
    #x,y
    [forward,descent/2], #FL
    [forward,descent/2], #FR
    
    [forward,descent/2], #HL
    [forward,descent]  #HR
])

FL_home = home + np.array([
    #x,y
    [forward,0], #FL
    [forward,0], #FR
    
    [forward,0], #HL
    [forward,0]  #HR
])

velocity = np.array([
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
], dtype=float)


acceleration = np.array([
    #front left
    20, #hip
    20, #knee
    #front right
    20, #hip
    20, #knee
    #hind left
    20, #hip
    20, #knee
    #hind right
    20, #hip
    20 #knee
], dtype = float)


leg_dict = {
"FR":0,
"FL":1,
"HR":2,
"HL":3
}

def lower_leg(leg, current):
    i = leg_dict[leg]

    current[i][1] = descent
    current[previous_leg(i)][1] = descent/2
    current[next_leg(i)][1] = descent/2
    current[opposite_leg(i)][1] = 0

    return coordinates

def next_leg(i):
    i = i + 1

    if i > 3:
        i = 0

    return i

def previous_leg(i):
    i = i - 1

    if i < 0:
        i = 3

    return i

def opposite_leg(i):
    if i > 1:
        return i - 2
    else:
        return i + 2