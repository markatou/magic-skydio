
def get_position_from_rpc(s):
    pos = s.split(":")[1] # Extract position data from string
    pos = pos.split("]")[0] # No "speed" text in output
    pos = pos[2:] # Remove leading square bracket and whitespace 
    converted_pos = [float(x) for x in pos.split(",")] # Convert string into float array
    return converted_pos

def get_speed_from_rpc(s):
    speed = s.split(":")[2] # Extract speed data from string
    speed = speed[:-1] # Remove trailing curly brace
    converted_speed = float(speed)
    return converted_speed
