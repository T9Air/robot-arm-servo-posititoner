import serial
import time
import cv2
import os

arduino = serial.Serial('COM5', 9600, timeout=1)

camera = cv2.VideoCapture(0)

images_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

number = 1

def capture_image():
    global number
    ret, frame = camera.read()
    if ret:
        filename = os.path.join(images_dir, f"image_{number}.jpg")        
        cv2.imwrite(filename, frame)
        print(f"Captured {filename}")
        number += 1
        if number == 4:
            number = 500
        arduino.write(b'1\n')
    else:
        print("Failed to capture image")

try:
    while True:
        if arduino.in_waiting > 0:
            command = arduino.readline().decode('utf-8').strip()
            if command == "go":
                print("Trigger received: Capturing image...")
                capture_image()
            elif command == "done":
                    camera.release()
                    arduino.close()
                    break
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    camera.release() # Release the camera
    arduino.close()