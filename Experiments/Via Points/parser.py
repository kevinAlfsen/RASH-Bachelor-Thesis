import numpy as np
import config
import kinematics

def is_num(n):
    
    '''
    Checks if input is a number (positive, negative, integer, float)
    
    Takes 1 input:
    n -> number to check, string or character
    
    Returns:
    False if n is not a number,
    True if n is a number
    '''
    
    try:
        float(n)
    except ValueError:
        return False
    return True

def parse_value(value):
    
    '''
    Parses a single term. Not designed to be called outside of parse_input
    
    Takes 1 input:
    value -> term to parse, needs to be on the form a*b/c*d, where a*b is the nominator and b*c is the denominator
    
    Returns:
    q -> float value of the parsed term
    '''
    
    
    try:
        nominator, denominator = value.split('/')
    except ValueError:
        nominator = value
        denominator = "1"
    
    nominator_factors = nominator.split('*')
    denominator_factors = denominator.split('*')
    
    nominator = 1
    denominator = 1
    for factor in nominator_factors:
        factor = factor.strip()
        if is_num(factor):
            nominator = nominator * float(factor)
        elif factor == 'pi' or factor == 'PI':
            nominator = nominator * np.pi
        elif factor == '-pi' or factor == '-PI':
            nominator = nominator * -np.pi
        else:
            print(f"[{factor}] not recoqnized, did you mean to seperate by '*' or '/' ?")
            return None
    
    for factor in denominator_factors:
        factor = factor.strip()
        if is_num(factor):
            denominator = denominator * float(factor)
        elif factor == 'pi' or factor == 'PI':
            denominator = denominator * np.pi
        elif factor == '-pi' or factor == '-PI':
            denominator = denominator * -np.pi
        else:
            print(f"[{factor}] not recoqnized, did you mean to seperate by '*' or '/' ?")
            return None
            
    q = nominator / denominator
    
    return q

def interpret_input(input_string, mode, current_position):
    
    '''
    Prompts and parses an input from the terminal.
    Input needs to be on the form : qn = term or q = term1, term2, ..., term8 (spaces are optional)
    Each term has the form a*b/c*d where a*b is the nominator and c*d is the denominator
    a, b, c, d can be positive or negative integers or floats or -pi or pi
    
    Passing qn (where n is an integer corresponing to the index in the joint parameter array [0, 1, ... 7]),
    sets that element to the provided term
    
    Passing q sets all elements
    
    Takes 1 input:
    prompt -> String to prompt user with when asking for input

    returns:
    joint_params -> array of 8 join parameters
    '''
    
    if input_string == "exit" or input_string == "Exit":
        return None, "Exit"

    elif "pos" in input_string or "Pos" in input_string:
        return None, "Position"

    elif input_string == "stand" or input_string == "Stand":
        return None, "Stand"

    elif input_string == "Home" or input_string == "home":
        return None, "Home"

    elif input_string == "Current" or input_string == "current":
        return None, "Current"

    elif input_string == "Walk" or input_string == "walk":
        return None, "Walk"

    elif input_string == "Idle" or input_string == "idle":
        return None, "Idle"
    elif "simple walk" in input_string:
        
        message, cycle_num = input_string.split(",")
        cycle_num = int(cycle_num)

        return cycle_num, "simple walk"

    elif "trot" in input_string:
        
        message, cycle_num = input_string.split(",")
        cycle_num = int(cycle_num)

        return cycle_num, "simple trot"

    elif "creep" in input_string:
        
        message, cycle_num = input_string.split(",")
        cycle_num = int(cycle_num)

        return cycle_num, "Creep"

    elif input_string == "send" or input_string == "Send":
        return None, "Send"

    elif input_string == "balance":
        return None, "Balance"
    
    else:
        try:
            key, value = input_string.split('=')
        except ValueError:
            print("Wrong format! Input should be a key and a value seperated by =")
            return None, "InputError"

    #Num joints is in config
    num_joints = config.num_joints

    #Determine number of points
    if '|' in value:
        points = value.split("|")
        num_points = len(points)
    else:
        points = value
        num_points = 1
    
    #Determine index
    index = -1
    for c in key:
        if c.isdigit():
            index = int(c)
    
    joint_variables = np.zeros((num_points, num_joints))
            
    if key[0] == 'q':
        if index == -1:
            print("Wrong key. Joint input should specify joints number. E.g. >q0 = pi")
            return None, "InputError"
        else:
            if num_points > 1:
                for i in range(num_points):
                    joint_variables[i, index] = parse_value(points[i])
            else:
                joint_variables[:,index] = parse_value(points)
            
            return joint_variables, ""

    elif key[0] == 'l':
        if mode == "Walk":
            zero_config = kinematics.walk_zero_config
            elbows = kinematics.walk_elbow
            joint_polarities = kinematics.walk_joint_polarities
        else:
            #print("Robot needs to be in walk mode to send coorindate. Change mode by typing >Walk")
            #return None, "InputError"

            zero_config = kinematics.idle_zero_config
            elbows = kinematics.idle_elbow
            joint_polarities = kinematics.idle_joint_polarities
            
        try:
            if index == -1: #All legs
                coordinates = np.zeros((4, 2))
                
                if num_points > 1: #With via
                    for i in range(num_points):
                        x, y = points[i].split(",")
                        
                        num_x = float(x)
                        num_y = float(y)
                        
                        for coordinate in coordinates:
                            coordinate[0] = num_x
                            coordinate[1] = num_y
                        
                        joint_variables[i] = kinematics.simple_walk_controller(coordinates, current_position, zero_config, joint_polarities, elbows)
                        
                    return joint_variables, ""
                    
                else: #No via
                    x, y = points.split(",")
                    
                    num_x = float(x)
                    num_y = float(y)
                    
                    for coordinate in coordinates:
                        coordinate[0] = num_x
                        coordinate[1] = num_y
                        
                    joint_variables = kinematics.simple_walk_controller(coordinates, current_position, zero_config, joint_polarities, elbows)
                        
                    return joint_variables, ""
                        
            else: #One leg
                coordinates = np.copy(config.home_coordinates)
                
                if num_points > 1: #With via
                    for i in range(num_points):
                        x, y = points[i].split(",")
                        
                        coordinates[index][0] = float(x)
                        coordinates[index][1] = float(y)
                        
                        joint_variables[i] = kinematics.simple_walk_controller(coordinates, current_position, zero_config, joint_polarities, elbows)
                    
                    return joint_variables, ""
                        
                else: #No via
                    x, y = points.split(",")
                    
                    coordinates[index][0] = float(x)
                    coordinates[index][1] = float(y)
                    
                    #print(coordinates)
                    
                    #print(joint_variables)
                    
                    joint_variables = kinematics.simple_walk_controller(coordinates, current_position, zero_config, joint_polarities, elbows)
                    
                    return joint_variables, ""
                
        except ValueError:
            print("Wrong format! Coordinates should look like: l = 100,-100 or l = -100,100")

            return None, "InputError" 
