import cv2
import numpy as np
import time
import serial
import pandas as pd
import datetime


# Variable initializations:
capture_duration = 60 # 90  # Duration for capturing video in seconds
cw_capture_period = 30 # 60
ccw_capture_period = 45 # 75

mcu_cw_write = True
mcu_ccw_write = True

cw_oneshot = True
ccw_oneshot = True

# Camera setup
fps = 6
camera_index = 1        # Set the camera index
camera = cv2.VideoCapture(camera_index) 

# Get the dimensions of the camera
video_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
video_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Serial setup
mcu = serial.Serial('COM5', 115200, timeout=1)  # Update port name as needed

# if mcu is None or not mcu.isOpened():
#     raise Exception('Warning: unable to open image source: CAMERA ', camera_index)

# Plot variables
capture_time_logger = np.array([])
motor_command_logger = np.array([])

# Counter variable to keep track of frame count
counter = 0

print("\nPress Enter to continue: ")
input()  # The script will pause here until Enter is pressed

print('\nStarting Trial ...\n')

# DEBUGGING: Initialize video export file for modified frames
# Create a video writer object to save the modified frames.
# Get the current time
current_time = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

debug_video = cv2.VideoWriter("LIVE_VIDEO_" + current_time + ".mp4", cv2.VideoWriter_fourcc(*"mp4v"),
                            6, (video_width, video_height))


timeStart = time.time()

#######################################################################################################
# INFERENCE LOOP
#######################################################################################################

while int(time.time() - timeStart) < capture_duration:

    start_time = time.time()

    # Get current camera frame
    ret, frame = camera.read()

    # Save writing timestamp
    current_daytime = datetime.datetime.now()
    time_combo = str(current_daytime.hour) + '-' + str(current_daytime.minute) + '-' + str(current_daytime.second) + '-' + str(current_daytime.microsecond)
    capture_time_logger = np.append(capture_time_logger, time_combo)

    # Write the frame to the output debug video.
    debug_video.write(frame)

    # Sending commands to mcu
    if (int(time.time() - timeStart) >= cw_capture_period) and (mcu_cw_write == True):
        # If time is greater than or equal to cw capture period
        # Send 'CW" command to Teensy
        if cw_oneshot == True:
            print('1. Sending CW Rotation Command!!!')
            mcu_text = 'CW'
            mcu.write(mcu_text.encode('ascii'))
            cw_oneshot = False

        motor_command_logger = np.append(motor_command_logger, 1)

    elif (int(time.time() - timeStart) >= ccw_capture_period) and (mcu_ccw_write == True):
        # If time is greater than or equal to cw capture period
        # Send 'CCW" command to Teensy
        if ccw_oneshot == True:
            print('2. Sending CCW Rotation Command!!!')
            mcu_text = 'CCW'
            mcu.write(mcu_text.encode('ascii'))
            ccw_oneshot = False
        
        motor_command_logger = np.append(motor_command_logger, -1)

    else:
        motor_command_logger = np.append(motor_command_logger, 0)

    # Keep checking if mcu sends 'COMPLETE' response
    if (mcu.inWaiting() > 0):

        print('\n*** MCU Sending Response ***')
        
        # read the bytes and convert from binary array to ASCII
        data_str = mcu.read(mcu.inWaiting()).decode('ascii')
        data_str = int(data_str)
        print('*** MCU Response Data: ', data_str, ' ***\n')

        # if data_str == 'COMPLETE-CW ':
        if data_str == 100:  
            mcu_cw_write = False
            print('1. CW Rotation Complete!')

        elif data_str == 101: 
            mcu_ccw_write = False
            print('2. CCW Rotation Complete!')

    # Get duration so far
    inference_time = time.time() - start_time

    # Some delay to slow down GPU inference; comment out
    if inference_time < 0.165:
        time_compensation = round((1/fps) - inference_time, 3)
        time.sleep(time_compensation)
        # print("Total tracking time: " + str(time_compensation + inference_time))


#######################################################################################################
# SAVE FILES TO DISK
#######################################################################################################

# Release image sources and video files
camera.release()
debug_video.release()

print("\nSAVING INFERENCE DATA TO DISK ...")

# Print array shapes
print('\nCapture TIme:       ', capture_time_logger.shape,
      '\nMotor Command:      ', motor_command_logger.shape)

# Get the current time
current_time = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

d1 = {'Capture Time': capture_time_logger,
      'Motor Command': motor_command_logger}

df1 = pd.DataFrame(d1)
        
# Save dataframe to CSV
df1.to_csv(f"VERIFICATION_DATA_{current_time}.csv")
