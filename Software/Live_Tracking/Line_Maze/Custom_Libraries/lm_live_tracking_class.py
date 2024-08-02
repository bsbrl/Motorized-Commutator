from dlclive import DLCLive, Processor
import cv2
import numpy as np
from Custom_Libraries.vector_functions import vectorAngle, animate_donut, write_read, donut_arrow, \
    rotations_over_time, debug_print, check_datatype, vectorAngle_v2, radius_filter
import time
import matplotlib.pyplot as plt
import serial
import pandas as pd
import datetime


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
                 dlc_display=False):
        
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

        self.tracking_verbose = verbose
        self.mcu_control = False
        
        self.theta = 0
        self.commutative_angle = 0
        self.frame_counter = 0
        self.rotations = 0

        # Flags to control the loop and pause state
        self.running = True
        self.paused = False
        
        self.angle_threshold = 225
        self.x_threshold = 75.
        self.tracking_fps = 10              # In frames per second
        self.tracking_period = 0.1          # In seconds
        # self.point_filter_radius = 100
        # self.points_filtered = 0

        self.v0 = [0, 0]
        self.v1 = [0, 0]
        self.lnut = []
        self.rnut = []
        self.meso = []
        self.tailbase = []
        self.mc_midpoint_offset = 50
        self.x_move_amount = 0
        self.angle_move_amount = 0

        self.x_move_timer = 0
        self.angle_move_timer = 0
        self.move_timer_offset = 300	# milliseconds
        self.m = 0.754
        self.b = 337.7
        self.STEPSIZE = 1

        self.segment_count = 8
        self.segment_number = int(self.segment_count / 2)
        self.left_offset = 0      # GOTTEN FROM IMAGEJ
        self.right_offset = 0     # GOTTEN FROM IMAGEJ
        
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
        
        self.rolling_angles = 0
        self.total_time_logger = np.array([])
        self.translation_data_logger =np.array([])
        self.angle_data_logger_1 = np.array([])
        self.angle_data_logger_2 = np.array([])

        self.meso_x = np.array([])
        self.meso_y = np.array([])
        self.tailbase_x = np.array([])
        self.tailbase_y = np.array([])

    
    def init_mcu(self):
        
        if self.COM_Port is not None and self.baudrate is not None:
            self.mcu = serial.Serial(port=self.COM_Port, baudrate=self.baudrate, timeout=1)
            self.mcu_control = True
            print("MCU Serial Connection Initialized.")
        else:
            self.mcu_control = False
            print("!!! MCU serial connection not initialized because PORT and BAUDRATE were not set.")
    
    def tracking_mcu(self, frame):
        x_move = 0
        angle_move = 0		# IN STEPS
        
        # self.theta is the angle between the new and previous vector
        # self.commutative_angle is the combined angle values over frames
        # self.frame_counter tracks the tacked/unskipped frames
        self.theta = vectorAngle_v2(self.v1, self.v0)
        debug_print(self.tracking_verbose, "\n*** Angle between the two vectors (degrees): " + str(np.rad2deg(self.theta)))
        
        self.commutative_angle += np.rad2deg(self.theta)
        debug_print(self.tracking_verbose, "*** Commutative angle: " + str(self.commutative_angle))
        
        if ((time.time() - self.angle_move_amount) > self.angle_move_timer):
            if (self.angle_move_amount != 0):
                self.angle_move_amount = 0
                self.angle_move_timer = 0
                debug_print(self.tracking_verbose, "--- ANGLE MOVE DONE! READY FOR NEXT ANGLE MOVE. ---")

            if abs(self.commutative_angle) >= self.angle_threshold:
                angle_move = int(self.commutative_angle)

                # Convert angle move to steps
                angle_move = int((angle_move * 200 * self.STEPSIZE) / 360.0)

                self.commutative_angle = 0
        
        # Increment frame counter
        self.frame_counter += 1
        
        if ((time.time() - self.x_move_amount) > self.x_move_timer):
            if (self.x_move_amount != 0):
                self.x_move_amount = 0
                self.x_move_timer = 0
                debug_print(self.tracking_verbose, "--- X MOVE DONE! READY FOR NEXT X MOVE. ---")

            # Check if mouse is out of defined x threshold
            if self.meso[0] < (self.video_width / 2):
                # Mouse is on the left side of the image frame
                # Calculations is done wrt the right nut
                # Normalize to the calibration image width: 1920
                x_gap = (self.rnut[0] - self.meso[0] - self.mc_midpoint_offset) * (1920 / self.video_width)
                debug_print(self.tracking_verbose, "x-gap (LEFT SIDE): " + str(x_gap))
                
                if abs(x_gap) > self.x_threshold:
                    # Generate x motor movement to send to mcu if mcu control is enabled
                    # x_move -> STEPS    x_gap -> PIXELS
                    x_move = int(x_gap * 0.54375)

            elif self.meso[0] >= (self.video_width / 2):
                # Mouse is on the right side of the image frame
                # Calculations is done wrt the left nut
                # Normalize to the calibration image width: 1920
                x_gap = (self.lnut[0] - self.meso[0] + self.mc_midpoint_offset) * (1920 / self.video_width)
                debug_print(self.tracking_verbose, "x-gap (RIGHT SIDE): " + str(x_gap))
                
                if abs(x_gap) > self.x_threshold:
                    # Generate x motor movement to send to mcu if mcu control is enabled
                    # x_move -> STEPS    x_gap -> PIXELS
                    x_move = int(x_gap * 0.54375)
        
        # Send rotation and translation to serial if mcu control is enabled and condition is met
        # Otherwise, just set the commutative value to zero
        if self.mcu_control and ((x_move != 0) or (angle_move != 0)):
            mcu_write_value = str(x_move) + ',' + str(angle_move)
            # mcu_value = write_read(mcu_write_value, self.mcu)
            debug_print(self.tracking_verbose, "MCU communication done ... \n")

            if (x_move != 0):
                self.x_move_amount = (self.m * x_move + self.b + self.move_timer_offset) * 0.001
                debug_print(self.tracking_verbose, "X MOVE DELAY AMOUNT (seconds): " + str(self.x_move_amount))
                self.x_move_timer = time.time()

            if (angle_move != 0):
                self.angle_move_amount = (self.m * angle_move + self.b + self.move_timer_offset) * 0.001
                debug_print(self.tracking_verbose, "ANGLE MOVE DELAY AMOUNT (seconds): " + str(self.angle_move_amount))
                self.angle_move_timer = time.time()
        
        elif (x_move != 0) or (angle_move != 0):
            mcu_write_value = str(x_move) + ',' + str(angle_move)
            debug_print(self.tracking_verbose, "MCU Write Value: " + str(mcu_write_value))
            
            if (x_move != 0):
                self.x_move_amount = (self.m * x_move + self.b + self.move_timer_offset) * 0.001
                debug_print(self.tracking_verbose, "X MOVE DELAY AMOUNT (seconds): " + str(self.x_move_amount))
                self.x_move_timer = time.time()

            if (angle_move != 0):
                self.angle_move_amount = (self.m * angle_move + self.b + self.move_timer_offset) * 0.001
                debug_print(self.tracking_verbose, "ANGLE MOVE DELAY AMOUNT (seconds): " + str(self.angle_move_amount))
                self.angle_move_timer = time.time()
            
        # Update rotations variable
        self.rotations = self.rotations + np.rad2deg(self.theta)

        debug_print(self.tracking_verbose, " ")
        debug_print(self.tracking_verbose, "x move:				" + str(x_move))
        debug_print(self.tracking_verbose, "x move timer: 		" + str(self.x_move_timer) + ", " + str(time.time() - self.x_move_timer))
        debug_print(self.tracking_verbose, "angle move: 		" + str(angle_move))
        debug_print(self.tracking_verbose, "angle move timer: 	" + str(self.angle_move_timer) + ", " + str(time.time() - self.angle_move_timer))
        debug_print(self.tracking_verbose, " ")

        # Adjust rotations to within [0, 360]
        if self.rotations < 0:
            self.rotations += 360
        elif self.rotations > 359:
            self.rotations -= 360
    
    def get_mouse_segment(self, mouse_x, num_segments, segment_width, gap_width):
        # Check first if the mouse is at the edge of the current segment
        segment_start = (self.segment_number - 1) * segment_width
        segment_end = self.segment_number * segment_width
        if (segment_start - gap_width) <= mouse_x < (segment_end + gap_width):
            print("Start: ", segment_start, ";  End: ", segment_end)
            return self.segment_number
        
        for segment in range(num_segments):
            segment_start = segment * segment_width
            segment_end = (segment + 1) * segment_width
            if (segment_start - gap_width) <= mouse_x < (segment_end + gap_width):
                print("Start: ", segment_start, ";  End: ", segment_end)
                return segment + 1
        return None
    
    def get_mouse_segment_offset(self, mouse_x, num_segments, segment_width, gap_width):
        # Check first if the mouse is at the edge of the current segment
        segment_start = ((self.segment_number - 1) * segment_width) + self.left_offset
        segment_end = (self.segment_number * segment_width) + self.left_offset
        if (segment_start - gap_width) <= mouse_x < (segment_end + gap_width):
            print("Start: ", segment_start, ";  End: ", segment_end)
            return self.segment_number
        
        for segment in range(num_segments):
            segment_start = (segment * segment_width) + self.left_offset
            segment_end = ((segment + 1) * segment_width) + self.left_offset
            if (segment_start - gap_width) <= mouse_x < (segment_end + gap_width):
                print("Start: ", segment_start, ";  End: ", segment_end)
                return segment + 1
        return 4    # STAY IN THE MIDDLE

    def tracking_mcu_segmented(self):
        
        ###################### ANGLE COMPUTATION ###########################################
        
        # self.theta is the angle between the new and previous vector
        # self.commutative_angle is the combined angle values over frames
        # self.frame_counter tracks the tacked/unskipped frames
        angle_move = 0		# IN STEPS

        self.theta = vectorAngle_v2(self.v0, self.v1)
        debug_print(self.tracking_verbose, "\n*** Angle between the two vectors (degrees): " + str(np.rad2deg(self.theta)))
        
        self.commutative_angle += np.rad2deg(self.theta)
        debug_print(self.tracking_verbose, "*** Commutative angle: " + str(self.commutative_angle))

        self.rolling_angles += np.rad2deg(self.theta)

        self.angle_data_logger_1 = np.append(self.angle_data_logger_1, self.commutative_angle)
        self.angle_data_logger_2 = np.append(self.angle_data_logger_2, self.rolling_angles)

        if abs(self.commutative_angle) >= self.angle_threshold:
            angle_move = int(self.commutative_angle)

            # Convert angle move to steps
            angle_move = int((angle_move * 200 * self.STEPSIZE) / 360.0)

            self.commutative_angle = 0

        ###################### TRANSLATION COMPUTATION ###########################################
        
        # Check which frame segment the mouse is
        # num_segments = 5
        # segment_width = self.video_width / self.segment_count
        segment_width = (self.video_width - self.left_offset - self.right_offset) / self.segment_count
        gap_width = segment_width * 0.20

        self.segment_number = self.get_mouse_segment_offset(self.meso[0], self.segment_count, segment_width, gap_width)
        # segment_number = (self.meso[0] + self.tailbase[0]) // (self.video_width * 2)
        debug_print(self.tracking_verbose, "*** Segment Number: " + str(self.segment_number) + "  ***\n")

        ###################### MCU COMMUNICATION ###########################################
        
        # Send rotation and translation to serial if mcu control is enabled and condition is met
        # Otherwise, just set the commutative value to zero
        if self.mcu_control and ((self.segment_number >= 0) or (angle_move != 0)):
            # Generate value to write to MCU
            mcu_write_value = str(self.segment_number) + ',' + str(angle_move) + '\n'
            # mcu_write_value = str(self.segment_number)

            # Send data via serial
            # self.mcu.write(mcu_write_value.encode())
            # self.mcu.write(bytes(mcu_write_value, 'utf-8'))
            res = write_read(mcu_write_value, self.mcu)

            debug_print(self.tracking_verbose, "MCU communication done ... \n")
        
        elif (self.segment_number >= 0) or (angle_move != 0):
            mcu_write_value = str(self.segment_number) + ',' + str(angle_move)
            debug_print(self.tracking_verbose, "MCU Write Value: " + str(mcu_write_value))

        ###################### UPDATE VARIABLES FOR STORAGE ###########################################
            
        # Update rotations variable
        self.rotations = self.rotations + np.rad2deg(self.theta)

        # Increment frame counter
        self.frame_counter += 1

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

            self.img_source = cv2.VideoCapture(self.camera_index)
            self.img_source = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
            self.img_source.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)             # 1920 | 960
            self.img_source.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)            # 1080 | 720
            
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

        # ARENA DEPENDENT OFFSETS
        # REFERENCE IMAGE RESOLUTION IS 1920 x 1080
        self.left_offset = int((400 / 1920) * self.video_width)      # GOTTEN FROM IMAGEJ | 350
        self.right_offset = int((150 / 1920) * self.video_width)     # GOTTEN FROM IMAGEJ | 270
    
    def inference_data_logger(self):
        # Log data
        self.inference_time_logger = np.append(self.inference_time_logger, self.inference_time)
        self.frame_inferenced_counter = np.append(self.frame_inferenced_counter, self.frame_counter)
        self.x_move_logger = np.append(self.x_move_logger, 1 if self.x_move_timer > 0 else 0)
        self.angle_move_logger = np.append(self.angle_move_logger, 1 if self.angle_move_timer > 0 else 0)

        # self.angle_data_logger_1 = np.append(self.angle_data_logger_1, self.commutative_angle)
        # self.angle_data_logger_2 = np.append(self.angle_data_logger_2, self.rolling_angles)
        self.translation_data_logger = np.append(self.translation_data_logger, self.segment_number)

        self.meso_x = np.append(self.meso_x, self.meso[0])
        self.meso_y = np.append(self.meso_y, self.meso[1])
        self.tailbase_x = np.append(self.tailbase_x, self.tailbase[0])
        self.tailbase_y = np.append(self.tailbase_x, self.tailbase[0])

    def inference_data_saver(self):
        # Save files to disk
        debug_print(self.tracking_verbose, "\nSAVING INFERENCE DATA TO DISK ...")

        # Get the current time
        current_time = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        # Create Pandas dataframe for rotations over time
        d1 = {'Inference Time': self.inference_time_logger,
                'Frame Counter': self.frame_inferenced_counter,
                'X Move': self.x_move_logger,
                'Angle Move': self.angle_move_logger,
                'Angle Data 1': self.angle_data_logger_1,
                'Angle Data 2': self.angle_data_logger_2,
                'X Data': self.translation_data_logger,
                'Total Time': self.total_time_logger,
                'Meso x': self.meso_x,
                'Meso y': self.meso_y,
                'Tailbase x': self.tailbase_x,
                'Tailbase y': self.tailbase_y}
        
        df1 = pd.DataFrame(d1)
        
        # Save dataframe to CSV
        df1.to_csv(f"INFERENCE_DATA_{current_time}.csv")

    def draw_on_frame(self, frame):
        # Draw a circle on the right nut
        radius = 10
        offset = self.video_width / (self.segment_count * 2)
                
        segment_draw_offset = (((self.video_width - self.left_offset - self.right_offset) / self.segment_count) * (self.segment_number - 1) + 
                        ((self.video_width - self.left_offset - self.right_offset) / self.segment_count) * self.segment_number +
                        self.left_offset * 2) / 2
        
        cv2.circle(frame, (int(segment_draw_offset), int(self.video_height / 2.5)), radius,
                    (255, 0, 0), thickness=4)
        
        # Draw lines from the circle to the boundaries
        cv2.line(frame, (int(segment_draw_offset - offset), int(self.video_height / 2.5)),
                    (int(segment_draw_offset + offset), int(self.video_height / 2.5)), (255, 0, 255), 4) 

        return frame
    
    def draw_tracking_points_on_frame(self, frame):
        # Draw a circle on the right nut
        radius = 4
        
        cv2.circle(frame, (int(self.meso[0]), int(self.meso[1])), radius,
                    (255, 0, 0), thickness=6)
        
        cv2.circle(frame, (int(self.tailbase[0]), int(self.tailbase[1])), radius,
                    (255, 0, 255), thickness=6)

        return frame

    def start_posing(self):
        
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
        
        # Counter variable to keep track of frame count
        counter = 0

        debug_video = None

        # If the image source is VID, you can set starting position
        if self.img_source_name == 'VID':
            frame_id = 30*15
            self.img_source.set(cv2.CAP_PROP_POS_FRAMES, frame_id)

            # DEBUGGING: Initialize video export file for modified frames
            # Create a video writer object to save the modified frames.
            current_time = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            debug_video = cv2.VideoWriter("DEBUG_VIDEO_" + current_time + ".mp4", cv2.VideoWriter_fourcc(*"mp4v"),
									    self.video_fps, (self.video_width, self.video_height))
        else:
            # DEBUGGING: Initialize video export file for modified frames
            # Create a video writer object to save the modified frames.
            # Get the current time
            current_time = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            debug_video = cv2.VideoWriter("LIVE_VIDEO_" + current_time + ".mp4", cv2.VideoWriter_fourcc(*"mp4v"),
									    6, (self.video_width, self.video_height))
            
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

        # print("\nPress Enter to continue: ")
        # input()  # The script will pause here until Enter is pressed

        # Rest of your code
        print("Script continues...\n")

        timeStart = time.time()
        
        # while loop and counter < self.frames_to_read:
        while self.img_source.isOpened() and counter < self.frames_to_read:

            # Print counter to track frames tracked
            debug_print(self.tracking_verbose, "Counter: " + str(counter))
            
            # Reading next frame from the camera
            ret, frame = self.img_source.read()
            # print("Result: ", result)
            
            # if frame is read correctly ret is True
            if not ret:
                debug_print(self.tracking_verbose, "Camera could not be found!")
                break
            
            start_time = time.time()
            
            # Get inference from an image
            img_pose = self.dlc_live.get_pose(frame)
            print("Image Pose: ", img_pose)
            
            # Get duration
            self.inference_time = time.time() - start_time
            debug_print(self.tracking_verbose, "--- %s seconds ---" % self.inference_time)
            
            self.lnut = img_pose[0]
            self.rnut = img_pose[1]
            self.meso = img_pose[2]
            self.tailbase = img_pose[3]
            
            # Check for NANS
            datatype_rotate = check_datatype(self.meso, self.tailbase)
            rotate_probability = True if ((self.meso[2] > 0.95) and (self.tailbase[2] > 0.95)) else False

            datatype_translate = check_datatype(self.lnut, self.rnut)
            translate_probability = True if ((self.lnut[2] > 0.95) and (self.rnut[2] > 0.95)) else False
            
            # if datatype_rotate and datatype_translate:
            # if rotate_probability and translate_probability:
            if rotate_probability:
                
                # create vector pointing from meso to tailbase
                x = self.tailbase[0] - self.meso[0]
                y = self.tailbase[1] - self.meso[1]
                
                # Calculate the horizontal midpoint of the nuts
                # mid_nut = (self.lnut[0] + self.rnut[0]) / 2
                
                if counter == 0:
                    # First frame in the video
                    self.v0 = [x, y]
                    # debug_print(self.tracking_verbose, "updated v0: " + str(self.v0))
                    
                    previous_meso = self.meso[:2]
                    previous_tailbase = self.tailbase[:2]
                
                else:
                    # Current frame's vector
                    self.v1 = [x, y]
                    # debug_print(self.tracking_verbose, "updated v1: " + str(self.v1))

                    # Perform tracking if there is no jump
                    # self.tracking_mcu(frame)
                    self.tracking_mcu_segmented()
                    
                    # Set previous vector to the new vector against next frame reading
                    self.v0 = self.v1[:]

                    if self.img_source_name == 'VID':
                        # Perform drawing
                        frame = self.draw_on_frame(frame)

                        # Write the modified frame to the output debug video.
                        debug_video.write(frame)
                    else:
                        # Draw tracking points on frame
                        # frame = self.draw_tracking_points_on_frame(frame)
                        
                        # Write the modified frame to the output debug video.
                        debug_video.write(frame)

                    # Log data
                    self.total_time_logger = np.append(self.total_time_logger, time.time() - timeStart)
                    self.inference_data_logger()
                    
                    previous_meso = self.meso[:2]
                    previous_tailbase = self.tailbase[:2]
                
                # Increment frame counter variable
                self.frame_counter += 1

                # increment counter
                counter = counter + 1
            
            else:
                debug_print(self.tracking_verbose, "!!! DATATYPE OR PROBABILITY PROBLEM")

            # increment counter
            # counter = counter + 1
            debug_print(self.tracking_verbose, "#################################################################")

            # Some delay to slow down GPU inference; comment out
            if self.inference_time < 0.165:
                time_compensation = 0.165 - round(self.inference_time, 3)
                time.sleep(time_compensation)
                debug_print(self.tracking_verbose, "Total tracking time: " + str(time_compensation + self.inference_time))
        
        # Give feedback that tracking is done
        if counter > 0:
            print("\nTracking Complete!\n")

        # Release image sources and video files
        self.img_source.release()
        
        if self.img_source_name == 'VID':
            debug_video.release()

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
    #     poser = dlclive_commutator(model_path, video_topose, plot_donut=True, skip_frames=10, frames_to_read=250)
    #     poser.start_posing()
    
    # else:
    #     poser = dlclive_commutator(model_path, camera_index=0, plot_donut=True, frames_to_read=10)
    #     poser.start_posing()
