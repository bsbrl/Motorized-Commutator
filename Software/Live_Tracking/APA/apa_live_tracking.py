from Custom_Libraries.apa_live_tracking_class import dlclive_commutator

# Define global variables

# DLC model path
dlc_config_modelpath = r"C:\Users\suhasa-lab\Downloads\DLC_APA_mobilenet"

# Video file path
video_file_name = r"C:\Users\VIDEO-PATH.mp4"

# Serial conection parameters
selected_serial_port = "COM5"
baud_rate = 115200						# 115200

# Camera and video
camera_index = 1
video_frame_to_skip = 0				# 3 to simulate 10 fps from 30 fps video
camera = True                       # True: live tracking   |  False: video simulation
tracking_fps = 10                   # Sets the live tracking rate | 6 is about minimum from tests and calculations
tracking_duration = 60 * 60 * tracking_fps      # In seconds

	
poser = dlclive_commutator(dlc_model_path=dlc_config_modelpath,
                            camera=camera,
                            camera_index=camera_index,
                            # video_path=video_file_name,           # Uncomment for video simulation
                            # skip_frames=video_frame_to_skip,      # Uncomment for video simulation
                            COM_Port=selected_serial_port,          # Comment for video simulation
                            baudrate=baud_rate,                     # Comment for video simulation
							tracking_duration=tracking_duration,
							verbose=True,
                            dlc_display=True,
                            save_tracking_video=True)

try:	
    poser.start_posing()

except Exception as e:
    print(e)
