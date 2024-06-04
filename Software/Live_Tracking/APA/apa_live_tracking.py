from Custom_Libraries.apa_live_tracking_class import dlclive_commutator

# Define global variables

# DLC model path
dlc_config_modelpath = r"C:\Users\kapil\Documents\DLC\IBrahim DLC\APA_2point_commutator-Kapil-2023-03-14\exported-models\DLC_APA_2point_commutator_resnet_50_iteration-3_shuffle-1"

# Video file path
video_file_name = r"C:\Users\kapil\Documents\DLC\IBrahim DLC\APA_2point_commutator-Kapil-2023-03-14\videos\Kapil Video Clipping_01.mp4"

# Serial conection parameters
selected_serial_port = "COM6"
baud_rate = 115200						# 115200
serial_timeout = 1
connect_arduino = True

# Camera and video
camera_index = 1
video_frame_to_skip = 0				# 3 to simulate 10 fps from 30 fps video
camera = True
tracking_fps = 10
inference_duration = 60 * 10      # In seconds

	
poser = dlclive_commutator(dlc_model_path=dlc_config_modelpath,
                            camera=camera,
                            camera_index=camera_index,
                            video_path=video_file_name,
                            skip_frames=video_frame_to_skip,
                            COM_Port=selected_serial_port,
                            baudrate=baud_rate,
							inference_duration=inference_duration,
							verbose=True,
                            dlc_display=True)

try:	
    poser.start_posing()

except Exception as e:
    print(e)
