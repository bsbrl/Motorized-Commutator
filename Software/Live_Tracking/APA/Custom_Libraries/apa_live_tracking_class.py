from dlclive import DLCLive, Processor
import cv2
import numpy as np
from Custom_Libraries.vector_functions import write_read, debug_print, check_datatype, vectorAngle_v2
import time
import matplotlib.pyplot as plt
import serial
import pandas as pd
import datetime
import keyboard
import math
# from pynput import keyboard


class dlclive_commutator():
    
    def __init__(self,
                 dlc_model_path,
                 camera=False,
                 camera_index=0,
                 video_path=None,
                 skip_frames=0, 
                 COM_Port=None, 
                 baudrate=None, 
                 tracking_duration=100,    # In seconds
                 verbose=True,
                 dlc_display=False,
                 save_tracking_video=False):
        
        self.img_source = None
        self.dlc_live = None
        self.mcu = None
        self.dlc_proc = None
        self.video=None
        
        self.dlc_model_path = dlc_model_path
        self.camera = camera
        self.camera_index = camera_index
        self.video_path = video_path
        self.skip_frames = skip_frames
        self.COM_Port = COM_Port
        self.baudrate = baudrate
        self.tracking_duration = tracking_duration
        self.dlc_display = dlc_display
        self.save_tracking_video = save_tracking_video

        self.tracking_verbose = verbose
        self.mcu_control = False
        
        self.theta = 0
        self.commutative_angle = 0
        self.frame_counter = 0
        self.skipped_frame_counter = 0
        self.rotations = 0
        self.commutation_status = 0
        
        self.angle_threshold = 90
        self.accuracy_threshold = 0.90
        self.tracking_fps = 10              # In frames per second
        self.tracking_period = 0.1          # In seconds
        # self.point_filter_radius = 100
        # self.points_filtered = 0

        # Flags to control the loop and pause state
        self.running = True
        self.paused = False
        
        self.v0 = [0, 0]
        self.v1 = [0, 0]
        self.lnut = []
        self.rnut = []
        self.meso = []
        self.tailbase = []
        self.angle_move_amount = 0
        self.angle_move_residual = 0
        # self.STEPSIZE = 1             # Based on mcu-stepper configuration
        
        self.img_source_name = None
        self.video_width = 0
        self.video_height = 0
        self.video_fps = 30
        self.video_num_frames = 0

        # Logger variables
        self.inference_time = 0;                    
        self.inference_time_logger = np.array([])
        self.frame_inferenced_counter = np.array([])
        self.x_move_logger = np.array([])
        self.angle_move_logger = np.array([])
        self.inference_skipped_frame_counter = np.array([])
        
        self.rolling_angles = 0
        self.total_time_logger = np.array([])
        self.translation_data_logger =np.array([])
        self.angle_data_logger_1 = np.array([])
        self.angle_data_logger_2 = np.array([])
        self.angle_residual_logger = np.array([])

        self.meso_x = np.array([])
        self.meso_y = np.array([])
        self.meso_accuracy = np.array([])

        self.tailbase_x = np.array([])
        self.tailbase_y = np.array([])
        self.tailbase_accuracy = np.array([])

        self.commutation = np.array([])

    
    def init_mcu(self):
        
        if self.COM_Port is not None and self.baudrate is not None:
            self.mcu = serial.Serial(port=self.COM_Port, baudrate=self.baudrate, timeout=1)
            self.mcu_control = True
            print("MCU Serial Connection Initialized.")
        else:
            self.mcu_control = False
            print("!!! MCU serial connection not initialized because PORT and BAUDRATE were not set.")


    def tracking_mcu(self):
        
        ###################### ANGLE COMPUTATION ###########################################
        
        # self.theta is the angle between the new and previous vector
        # self.commutative_angle is the combined angle values over frames
        # self.frame_counter tracks the tacked/unskipped frames
        angle_move = 0		# IN STEPS
        self.angle_move_amount = 0
        self.commutation_status = 0

        self.theta = vectorAngle_v2(self.v0, self.v1)   # uncomment to change commutator rotation direction
        # self.theta = vectorAngle_v2(self.v1, self.v0) # comment to change commutator rotation direction
        debug_print(self.tracking_verbose, "\n*** Angle between the two vectors (degrees): " + str(np.rad2deg(self.theta)))
        
        self.commutative_angle += np.rad2deg(self.theta)
        debug_print(self.tracking_verbose, "*** Commutative angle: " + str(self.commutative_angle))

        self.rolling_angles += np.rad2deg(self.theta)

        # self.angle_data_logger_1 = np.append(self.angle_data_logger_1, self.commutative_angle)
        # self.angle_data_logger_2 = np.append(self.angle_data_logger_2, self.rolling_angles)

        if abs(self.commutative_angle) >= self.angle_threshold:
            # Convert commutative angle to integer and save the residual part
            # angle_move = int(self.commutative_angle)
            residual, angle_move = math.modf(self.commutative_angle)
            self.angle_move_residual += residual
            self.angle_move_amount = angle_move
            
            if abs(self.angle_move_residual) >= 10:
                residual, self.angle_move_residual = math.modf(self.angle_move_residual)
                self.angle_move_amount += self.angle_move_residual
                self.angle_move_residual = residual

            # CONVERT ANGLE MOVE TO STEPS:
            # angle_move = int((angle_move * 200 * self.STEPSIZE) / 360.0)

            self.commutative_angle = 0
            self.commutation_status = 1

        ###################### MCU COMMUNICATION ###########################################
        
        # Send rotation and translation to serial if mcu control is enabled and condition is met
        # Otherwise, just set the commutative value to zero
        if self.mcu_control and (angle_move != 0):
            # Generate value to write to MCU
            mcu_write_value = str(angle_move) + '\n'

            # Send data via serial
            res = write_read(mcu_write_value, self.mcu)

            debug_print(self.tracking_verbose, "MCU communication done ... \n")

        elif (angle_move != 0):
            # Generate value to write to MCU
            mcu_write_value = str(angle_move) + '\n'

            debug_print(self.tracking_verbose, "MCU Write Value: " + str(mcu_write_value))

        ###################### UPDATE VARIABLES FOR STORAGE ###########################################
            
        # Update rotations variable
        self.rotations = self.rotations + np.rad2deg(self.theta)

        # Adjust rotations to within [0, 360]
        if self.rotations < 0:
            self.rotations += 360
        elif self.rotations > 359:
            self.rotations -= 360


    def init_dlc_processor(self):
        self.dlc_proc = Processor()
        print("DLC Processor Initialized.")


    def init_dlclive_object(self):
        tf_config = None
        resize = 0.5
        cropping = None
        dynamic = (False, 0.5, 10)
        pcutoff = 0.5
        display_radius = 4
        display_cmap = 'bmy'

        self.dlc_live = DLCLive(self.dlc_model_path,
                                display=self.dlc_display,
                                pcutoff=pcutoff,
                                resize=resize,
                                display_radius=display_radius,
                                 display_cmap=display_cmap,
                                dynamic=dynamic,
                                cropping=cropping,
                                tf_config=tf_config)
        
        print("DLC Live Object Initialized.")


    def init_img_source(self):
        
        if self.camera:
            self.img_source_name = "CAM"

            # self.img_source = cv2.VideoCapture(self.camera_index)
            self.img_source = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
            self.img_source.set(cv2.CAP_PROP_FRAME_WIDTH, 960)             # 1920 | 960
            self.img_source.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)            # 1080 | 720

            if self.img_source is None or not self.img_source.isOpened():
                raise Exception('Warning: unable to open image source: CAMERA ', self.camera_index)
        
            else:
                # Get the dimensions of the camera
                self.video_width = int(self.img_source.get(cv2.CAP_PROP_FRAME_WIDTH))
                self.video_height = int(self.img_source.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
                # Reading a single frame from the camera
                ret, frame = self.img_source.read()
        
                # PRINT IMAGE DETAILS
                if self.tracking_verbose:
                    # height, width, number of channels in image
                    height = frame.shape[0]
                    width = frame.shape[1]
                    channels = frame.shape[2]
                    
                    # get dimensions of image
                    dimensions = frame.shape
                    
                    print("CAMERA DETAILS: ")
                    print('Image Dimension    : ', dimensions)
                    print('Image Height       : ', height)
                    print('Image Width        : ', width)
                    print('Number of Channels : ', channels)
                    print('')
        
        else:
            self.img_source_name = "VID"
            
            self.img_source = cv2.VideoCapture(self.video_path)

            if self.img_source is None or not self.img_source.isOpened():
                raise Exception('Warning: unable to open image source: VIDEO')
        
            if self.tracking_verbose:
                # Get the video properties.
                self.video_width = int(self.img_source.get(cv2.CAP_PROP_FRAME_WIDTH))
                self.video_height = int(self.img_source.get(cv2.CAP_PROP_FRAME_HEIGHT))
                self.video_fps = int(self.img_source.get(cv2.CAP_PROP_FPS))
                self.video_num_frames = int(self.img_source.get(cv2.CAP_PROP_FRAME_COUNT))
                
                # Print the video details.
                print("VIDEO DETAILS: ")
                print("Video width:", self.video_width)
                print("Video height:", self.video_height)
                print("Video fps:", self.video_fps)
                print("Number of frames:", self.video_num_frames)
                print(" ")
                
            print("Video Capture Initialized.\n")

  
    def inference_data_logger(self, timeStart, frame_skipping):
        if self.paused:
            self.frame_counter = 0
            self.meso = [0, 0, 0]
            self.tailbase = [0, 0, 0]


        # Log data
        self.inference_time_logger = np.append(self.inference_time_logger, self.inference_time)
        self.frame_inferenced_counter = np.append(self.frame_inferenced_counter, self.frame_counter)
        self.angle_move_logger = np.append(self.angle_move_logger, self.angle_move_amount)
        
        self.meso_x = np.append(self.meso_x, self.meso[0])
        self.meso_y = np.append(self.meso_y, self.meso[1])
        self.meso_accuracy = np.append(self.meso_accuracy, self.meso[2])

        self.tailbase_x = np.append(self.tailbase_x, self.tailbase[0])
        self.tailbase_y = np.append(self.tailbase_y, self.tailbase[1])
        self.tailbase_accuracy = np.append(self.tailbase_accuracy, self.tailbase[2])

        self.total_time_logger = np.append(self.total_time_logger, (time.time() - timeStart))
        self.inference_skipped_frame_counter = np.append(self.inference_skipped_frame_counter, self.skipped_frame_counter)

        self.angle_data_logger_1 = np.append(self.angle_data_logger_1, self.commutative_angle)
        self.angle_data_logger_2 = np.append(self.angle_data_logger_2, self.rolling_angles)
        self.angle_residual_logger = np.append(self.angle_residual_logger, self.angle_move_residual)

        self.commutation = np.append(self.commutation, self.commutation_status)


    def inference_data_saver(self):
        # Save files to disk
        debug_print(self.tracking_verbose, "\nSAVING INFERENCE DATA TO DISK ...")

        # Get the current time
        current_time = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        # Print array shapes
        print('\nInference time size:       ', self.inference_time_logger.shape,
              '\nFrame Counter:             ', self.frame_inferenced_counter.shape,
              '\nSkipped Frame Counter:     ', self.inference_skipped_frame_counter.shape,
              '\nAngle Move:                ', self.angle_move_logger.shape,
              '\nAngle Data 1:              ', self.angle_data_logger_1.shape,
              '\nAngle Data 2:              ', self.angle_data_logger_2.shape,
              '\nResidual angle:            ', self.angle_residual_logger.shape,
              '\nTotal Time:                ', self.total_time_logger.shape,
              '\nMeso x:                    ', self.meso_x.shape,
              '\nMeso y:                    ', self.meso_y.shape,
              '\nMeso accuracy:             ', self.meso_accuracy.shape,
              '\nTailbase x:                ', self.tailbase_x.shape,
              '\nTailbase y:                ', self.tailbase_y.shape,
              '\nTailbase accuracy:         ', self.tailbase_accuracy.shape,
              '\nCommutation:               ', self.commutation.shape)

        # Create Pandas dataframe for rotations over time
        d1 = {'Inference Time': self.inference_time_logger,
                'Frame Counter': self.frame_inferenced_counter,
                'Skipped FC': self.inference_skipped_frame_counter,
                'Angle Move': self.angle_move_logger,
                'Angle Data 1': self.angle_data_logger_1,
                'Angle Data 2': self.angle_data_logger_2,
                'Residual angle': self.angle_residual_logger,
                'Total Time': self.total_time_logger,
                'Meso x': self.meso_x,
                'Meso y': self.meso_y,
                'Meso accuracy': self.meso_accuracy,
                'Tailbase x': self.tailbase_x,
                'Tailbase y': self.tailbase_y,
                'Tailbase accuracy': self.tailbase_accuracy,
                'Commutation': self.commutation}
        
        df1 = pd.DataFrame(d1)
        
        # Save dataframe to CSV
        df1.to_csv(f"INFERENCE_DATA_{current_time}.csv")


    def draw_tracking_points_on_frame(self, frame):
        # Draw a circle on the right nut
        radius = 4
        
        cv2.circle(frame, (int(self.meso[0]), int(self.meso[1])), radius,
                    (255, 0, 0), thickness=6)
        
        cv2.circle(frame, (int(self.tailbase[0]), int(self.tailbase[1])), radius,
                    (255, 0, 255), thickness=6)

        return frame
    

    def toggle_pause(self, event):
        # Function toggles the pause state
        self.paused = not self.paused
        debug_print(self.tracking_verbose, f"Paused: {self.paused}")

        # Save data to disk everytime we pause
        self.inference_data_saver()

        # Reset frame counter
        self.frame_counter = 0

    
    def stop_loop(self, event):
        # Function stops the loop
        self.running = False


    def start_posing(self):

        # Set up the keyboard event listener
        keyboard.on_press_key("Home", self.toggle_pause)  # 'Home' key to pause/resume

        # Set up the keyboard event listener for stopping the loop
        keyboard.on_press_key("End", self.stop_loop)  # 'End' key to stop the loop
        
        # CALL INITIALIZATION FUNCTIONS
        
        # DEBUG PRINT
        print("\nCalling initialization functions ...\n")
        
        self.init_mcu()
        self.init_dlc_processor()
        self.init_dlclive_object()
        self.init_img_source()
        
        # DEBUG PRINT	
        print("Initialization successful ...\n")
        
        # DEFINE VARIABLES FOR POSE COMPUTATION
        frame_skipping = (self.video_fps / self.tracking_fps)       # Used for video simulation
        debug_video = None

        # If the image source is VID, you can set starting position
        if self.img_source_name == 'VID':
            # Reset frame pointer to first frame
            self.img_source.set(cv2.CAP_PROP_POS_FRAMES, 0)

            # DEBUGGING: Initialize video export file for modified frames
            # Create a video writer object to save the modified frames.
            current_time = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            
            debug_video = cv2.VideoWriter("DEBUG_VIDEO_" + current_time + ".mp4", cv2.VideoWriter_fourcc(*"mp4v"),
									    self.tracking_fps, (int(self.video_width/3), int(self.video_height/3)))

        else:
            # DEBUGGING: Initialize video export file for modified frames
            # Create a video writer object to save the modified frames.
            # Get the current time
            current_time = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

            if (self.save_tracking_video == True):
                live_video = cv2.VideoWriter("LIVE_VIDEO_" + current_time + ".mp4", cv2.VideoWriter_fourcc(*"mp4v"),
                                            self.tracking_fps, (int(self.video_width), int(self.video_height)))

            # Set buffer size
            self.img_source.set(cv2.CAP_PROP_BUFFERSIZE, 10)
        
        print("\nPress Enter to continue: ")
        input()  # The script will pause here until Enter is pressed
        
        # Reading a single frame from the image source
        ret, frame = self.img_source.read()
        
        # if frame is read correctly ret is True
        if not ret:
            raise Exception("\nCan't receive frame (stream end?). Exiting ...\n")

        # Init DLC inference
        self.dlc_live.init_inference(frame)

        print("\nInitializing Pose Tracking ...\n")
        print("\nPress 'Home' to pause/resume, 'End' to quit.\n")

        timeStart = time.time()
        loop_counter = 0

        while self.img_source.isOpened() and ((time.time() - timeStart) <= self.tracking_duration) and self.running:

            if not self.paused:

                loop_start_time = time.time()
                
                # Print counter to track frames tracked
                debug_print(self.tracking_verbose, "Loop counter:  " + str(loop_counter))
                debug_print(self.tracking_verbose, "Frame counter: " + str(self.frame_counter))

                # temp_time = time.time()

                # Reading next frame from the camera
                ret, frame = self.img_source.read()

                # print(time.time() - temp_time)
                # print("Image acquisition speed (seconds): ",  time.time() - temp_time)
                
                # if frame is read correctly ret is True
                if not ret:
                    debug_print(self.tracking_verbose, "Camera could not be found!")
                    break
                
                # Get inference from an image
                img_pose = self.dlc_live.get_pose(frame)
                # print("Image Pose: ", img_pose)
                
                # Get duration
                self.inference_time = time.time() - loop_start_time
                debug_print(self.tracking_verbose, "--- %s seconds ---" % self.inference_time)
                
                self.lnut = img_pose[0]
                self.rnut = img_pose[1]
                self.meso = img_pose[0]
                self.tailbase = img_pose[1]
                
                # Check for NANS
                datatype_rotate = check_datatype(self.meso, self.tailbase)
                rotate_accuracy = True if ((self.meso[2] > self.accuracy_threshold) and (self.tailbase[2] > self.accuracy_threshold)) else False
                
                if rotate_accuracy and datatype_rotate:
                    
                    # create vector pointing from meso to tailbase
                    x = self.tailbase[0] - self.meso[0]
                    y = self.tailbase[1] - self.meso[1]
                    
                    if self.frame_counter == 0:
                        # First frame in the video
                        self.v0 = [x, y]
                        # debug_print(self.tracking_verbose, "updated v0: " + str(self.v0))
                        
                        previous_meso = self.meso[:2]
                        previous_tailbase = self.tailbase[:2]
                    
                    else:
                        # Current frame's vector
                        self.v1 = [x, y]
                        # debug_print(self.tracking_verbose, "updated v1: " + str(self.v1))
                        
                        self.tracking_mcu()
                        
                        # Set previous vector to the new vector against next frame reading
                        self.v0 = self.v1[:]

                        if self.img_source_name == 'VID':
                            # Perform drawing
                            # frame = self.draw_on_frame(frame)
                            frame = self.draw_tracking_points_on_frame(frame)

                            # Write the modified frame to the output debug video.
                            debug_video.write(cv2.resize(frame, (int(self.video_width/3), int(self.video_height/3))))

                        else:
                            # Uncomment to draw tracking points on exported video frames
                            # frame = self.draw_tracking_points_on_frame(frame)
                            
                            if (self.save_tracking_video == True):
                                # Write the modified frame to the output debug video.
                                live_video.write(cv2.resize(frame, (int(self.video_width), int(self.video_height))))
                                # print(" ")

                        previous_meso = self.meso[:2]
                        previous_tailbase = self.tailbase[:2]

                    # Increment frame counter variable every time a frame is read
                    self.frame_counter += 1

                    self.skipped_frame_counter = 0
                
                else:
                    debug_print(self.tracking_verbose, "!!! DATATYPE OR PROBABILITY PROBLEM")
                    self.skipped_frame_counter = 1

                # Increment loop counter
                loop_counter += 1

                # Log data
                self.inference_data_logger(timeStart, frame_skipping)

                if self.img_source_name == 'VID':
                    # Simulate frame skipping
                    self.img_source.set(cv2.CAP_PROP_POS_FRAMES, loop_counter * frame_skipping)

                # Print duration of loop
                debug_print(self.tracking_verbose, "Current duration: " + str(time.time() - timeStart) + " seconds.")

                # Some delay to slow down GPU inference
                # 0.165 for 6 FPS
                # 0.1   for 10 FPS
                if (time.time() - loop_start_time) < self.tracking_period:
                    time_compensation = self.tracking_period - round((time.time() - loop_start_time), 3)
                    time.sleep(time_compensation)
                    debug_print(self.tracking_verbose, "Total tracking time: " + str(time_compensation + self.inference_time) + "\n\n")

            else:
                # Check every second if paused
                time.sleep(1)
        

        # Give feedback once tracking is done
        print("\nTracking Complete!\n")
        print("\nTotal number of skipped frames: ", np.sum(self.inference_skipped_frame_counter), "\n")

        # Release image sources and video files
        self.img_source.release()
        
        if self.img_source_name == 'VID':
            debug_video.release()

        if ((self.save_tracking_video == True) and (self.img_source_name == 'CAM')):
            live_video.release()

        # Save data to disk
        self.inference_data_saver()
        
        # Sleep for 5 seconds to display final results
        time.sleep(5.0)
        
        # Close open figure
        plt.close()


if __name__ == '__main__':
    
    # Define model path (to config.yaml)
    model_path = r"H:\Other computers\My PC\PhD_UOM\General\BSBRL\Projects\Motorized_Commutator\DLC\Projects\DLC-AUG-26-02--Ibrahim-2022-08-26\exported-models\DLC_DLC-AUG-26-02-_mobilenet_v2_1.0_iteration-0_shuffle-1"
    
    # Define video path
    video_topose = r"C:\Users\me-fausn002-admin\Downloads\Barnez maze 9 point tracking-Daniel-2021-04-19\videos\Mouse 405 T9 Cut Res2.mp4"
    
    # camera = True
    
    # if not camera:
    #     poser = dlclive_commutator_video(model_path, video_topose, skip_frames=10, frames_to_read=250)
    #     poser.start_posing()
    
    # else:
    #     poser = dlclive_commutator_camera(model_path, camera_index=0, frames_to_read=10)
    #     poser.start_posing()
