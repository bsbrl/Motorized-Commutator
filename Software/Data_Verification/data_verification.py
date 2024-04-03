import cv2
import serial
import time
import datetime
import csv
import threading

# Camera setup
fps = 6
capture_duration = 10  # Duration for capturing video in seconds
camera = cv2.VideoCapture(0)  # Change '0' to the correct camera index

# Serial setup
ser = serial.Serial('COM3', 115200, timeout=1)  # Update port name as needed

# Variables for commands
cw_command_duration = 10  # Duration before sending 'CW' command
ccw_command_duration = 20  # Duration before sending 'CCW' command

# Function to handle serial communication
def handle_serial_command(command, delay):
    ser.write(command.encode())
    time.sleep(delay)
    while True:
        if ser.readline().decode().strip() == 'COMPLETE':
            break
    time.sleep(5)  # Wait for 5 seconds
    ser.write(command.encode())  # Send command again

# Start capturing video
start_time = time.time()
with open('timestamps.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    while int(time.time() - start_time) < capture_duration:
        ret, frame = camera.read()
        if ret:
            # Save frame
            cv2.imwrite(f'frame_{int(time.time() - start_time)}.jpg', frame)
            # Log timestamp
            writer.writerow([datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')])

        time.sleep(1 / fps)

        # Send CW command after specified duration
        if int(time.time() - start_time) == cw_command_duration:
            threading.Thread(target=handle_serial_command, args=('CW', 5)).start()
        
        # Send CCW command after specified duration
        if int(time.time() - start_time) == ccw_command_duration:
            threading.Thread(target=handle_serial_command, args=('CCW', 5)).start()


# Release resources
camera.release()
ser.close()
