import serial
import math
import time

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

def cos_neg1(out, b, c):
    angle = (b**2 + c**2 - out**2) / (2 * b * c)
    angle = max(min(angle, 1), -1)
    return math.degrees(math.acos(angle))

def calculate_angles(x, y, z, arm1length, arm2length):
    # Calculate the angles of the joints
    
    # Calculate the angle of the base joint
    radiusXY = math.sqrt(x**2 + y**2)
    hypotenuseXY = math.sqrt((radiusXY - x)**2 + y**2)
    joint_1 = cos_neg1(hypotenuseXY, x, y)

    # Calculate the angle of the arm joints
    baseToEffector = math.sqrt(radiusXY**2 + z**2)
    angleA = cos_neg1(arm1length, baseToEffector, arm2length)
    angleB = cos_neg1(arm2length, arm1length, baseToEffector)
    angleC = 180 - (angleA + angleB)

    joint_2 = 180 - (math.tan(z / radiusXY) + angleB)
    joint_3 = 270 - angleC

    return joint_1, joint_2, joint_3

def send_angles(ser, joint_1, joint_2, joint_3):
    for attempt in range(3):
        recieved = False
        start_time = time.time()

        # Send the angles to the arduino
        message = f"{joint_1}, {joint_2}, {joint_3}"
        ser.write(message.encode('utf-8'))

        # Wait for the arduino to send the data back
        while not recieved and (time.time() - start_time) < .5:
            recieved = ser.readline().decode('utf-8').strip()
            if recieved == f"Received: {joint_1}, {joint_2}, {joint_3}":
                print(f"Success on attempt {attempt + 1}")
                return True
            time.sleep(0.1)

        print(f"Attempt {attempt + 1} failed, retrying...")
    
    print("Failed to send the angles to the arduino")
    return False

def main():
    ser = serial.Serial('COM3', 9600)
    ser.open()

    arm1length = 100 # length of the first arm in mm
    arm2length = 100 # length of the second arm in mm

    x = 0 # initial x coordinate: change to actual initial position
    y = 0 # initial y coordinate: change to actual initial position
    z = 0 # initial z coordinate: change to actual initial position

    joint_1 = 90 # rotate on z axis
    joint_2 = 90 # base joint
    joint_3 = 90 # first arm joint
    
    while True:
        positions = get_xyz_input(x, y, z)
        if not positions:
            ser.close()
            break
        x, y, z = positions
        joint_1, joint_2, joint_3 = calculate_angles(x, y, z, arm1length, arm2length)
        if not send_angles(ser, joint_1, joint_2, joint_3):
            print("Cannot send the angles to the arduino")
            print("Check the connection and restart the program")
            ser.close()
            break