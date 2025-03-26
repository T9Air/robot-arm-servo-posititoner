import serial

#################################################################
# 1. Add code for calculating angles of the joints
# 2. Add code for sending angles to the arduino
# 3. Add code for receiving data from the arduino
#################################################################

def get_xyz_input(prev_x, prev_y, prev_z):
    print("Press 'q' to quit, and 's' to keep the axis in the same position")
    x = input("Enter the x coordinate: ")
    if x == 'q':
        return False
    elif x == 's':
        x = prev_x
    y = input("Enter the y coordinate: ")
    if y == 'q':
        return False
    elif y == 's':
        y = prev_y
    z = input("Enter the z coordinate: ")
    if z == 'q':
        return False
    elif z == 's':
        z = prev_z
    return x, z

def calculate_angles(x, y, z):
    # calculate the angles of the joints
    return 

def main():
    ser = serial.Serial('COM3', 9600)
    
    x = 0 # initial x coordinate: change to actual initial position
    y = 0 # initial y coordinate: change to actual initial position
    z = 0 # initial z coordinate: change to actual initial position

    joint_1 = 90 # rotate on z axis
    joint_2 = 90 # base joint
    joint_3 = 90 # first arm joint
    
    while True:
        positions = get_xyz_input(x, z)
        if positions == False:
            break
        x, z = positions
        joint_1, joint_2, joint_3 = calculate_angles(x, y, z)
