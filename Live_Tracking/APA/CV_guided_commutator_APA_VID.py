from Custom_Libraries.dlclive_commutator_APA_VID import dlclive_commutator

# Define global variables

# DLC model path

# dlc_config_modelpath = r"DLC_Model\kp-OCT-18-23-apa-resnet50"
dlc_config_modelpath = r"DLC_Model\kp-OCT-18-23-bm-resnet50"

# Video file path
# video_file_name = r"Video\trim_1739_trial1-09112023113131-0000_115-221_sec.avi"
# video_file_name = r"Video\Kapil_Video_Clipping_01.mp4"
video_file_name = r"C:\Users\me-fausn002-admin\Downloads\APA_VIDS_DFF\trim_1739_d2_t1-05232023093343-0000_120-719.2595_sec.avi"

# Serial conection parameters
selected_serial_port = "COM6"
baud_rate = 115200						# 115200
serial_timeout = 1
connect_arduino = True

# Camera and video
camera_index = 1
video_frame_to_skip = 0				# 10 to simulate 3 fps
camera = False
img_frames_to_read = 6 * 60 * 20 # 60 * 5 # 6 * 15 | 30*60*2 | 1801 for 1 minute

	
poser = dlclive_commutator(dlc_model_path=dlc_config_modelpath,
                            camera=camera,
                            camera_index=camera_index,
                            video_path=video_file_name,
                            skip_frames=video_frame_to_skip,
                            # COM_Port=selected_serial_port,
                            # baudrate=baud_rate,
							plot_donut=False,
							frames_to_read=img_frames_to_read,
							verbose=True,
                            dlc_display=True)

try:	
    poser.start_posing()

except Exception as e:
    print(e)
