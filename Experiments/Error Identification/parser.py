import numpy as np

class ParseError(RuntimeError):
    def __init__(self, msg):
        self.msg = msg

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
            raise ParseError(f"[{factor}] not recoqnized, did you mean to seperate by '*' or '/' ?")
    
    for factor in denominator_factors:
        factor = factor.strip()
        if is_num(factor):
            denominator = denominator * float(factor)
        elif factor == 'pi' or factor == 'PI':
            denominator = denominator * np.pi
        elif factor == '-pi' or factor == '-PI':
            denominator = denominator * -np.pi
        else:
            raise ParseError(f"[{factor}] not recoqnized, did you mean to seperate by '*' or '/' ?")
    
    q = nominator / denominator
    
    return q

def parse_input(prompt=""):
    
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
    
    input_params = []
    joint_params = [0.0 for i in range(8)]

    input_params = input(prompt)

    if input_params == "exit" or input_params == "Exit":
        raise ParseError("Exit")

    if "pos" in input_params or "Pos" in input_params:
        raise ParseError("Position")
    
    if "cal" in input_params or "Cal" in input_params or input_params == "c":
        raise ParseError("Calibrate")

    try:
        key, value = input_params.split('=')
    except ValueError:
        raise ParseError("Wrong format! Input should be a key and a value seperated by =")
    
    index = -1
    
    for c in key:
        if c.isdigit():
            index = int(c)
            
    if index != -1:
        joint_params[index] = parse_value(value)
    else:
        values = value.split(',')
        for i in range(len(joint_params)):
            
            try:
                joint_params[i] = parse_value(values[i])
            except IndexError:
                raise ParseError("Wrong number of arguments! Value should contain 8 terms seperated by ,")

    if None in joint_params:
        raise ParseError("Joint params contains None for some reason?")
    else:
        return np.array(joint_params)