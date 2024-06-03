from Custom_Libraries.lm_live_tracking_dualCam_class import dlclive_commutator

# Define global variables

# DLC model path
dlc_config_modelpath = r"DLC_Model\ib-AUG-21-23_01-mobilenet"

# Video file path
# video_file_name = r"C:\Users\VIDEO-PATH.mp4"

# Serial conection parameters
selected_serial_port = "COM4"
baud_rate = 115200						# 115200
serial_timeout = 1
connect_arduino = True

# Camera and video parameters
camera_index = 0
video_frame_to_skip = 0             # For video simulation
camera = True
img_frames_to_read = 6 * 60 * 5     

	
poser = dlclive_commutator(dlc_model_path=dlc_config_modelpath,
                            camera=camera,
                            camera_index=camera_index,
                            # video_path=video_file_name,           # Uncomment for video simulation
                            # skip_frames=video_frame_to_skip,      # Uncomment for video simulation
                            COM_Port=selected_serial_port,          # Comment for video simulation
                            baudrate=baud_rate,                     # Comment for video simulation
							plot_donut=False,
							frames_to_read=img_frames_to_read,
							verbose=True,
                            dlc_display=True)

try:	
    poser.start_posing()

except Exception as e:
    print(e)
