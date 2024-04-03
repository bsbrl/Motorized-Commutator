from Custom_Libraries.dlclive_commutator_gantry_unified_v2_VID import dlclive_commutator

# Define global variables

# DLC model path
# dlc_config_modelpath = r"H:\Other computers\My PC\PhD_UOM\General\BSBRL\Projects\Motorized_Commutator\Python\DLC_Models\ib-AUG-21-23_01-mobilenet"
# dlc_config_modelpath = r"C:\Users\me-fausn002-admin\Downloads\Small_MC_PC\DLC_Model_Script_Video\DLC_Model\ib-AUG-21-23_01-mobilenet"
# dlc_config_modelpath = r"G:\Other computers\My PC\PhD_UOM\General\BSBRL\Projects\Motorized_Commutator\Python\DLC_Models\ib-AUG-21-23_01-mobilenet"
dlc_config_modelpath = r"DLC_Model\ib-AUG-21-23_01-mobilenet"
# dlc_config_modelpath = r"DLC_Model\kp-AUG-14-23-gantry-mobilenet"

# Video file path
# video_file_name = r"C:\Users\me-fausn002-admin\Downloads\Small_MC_PC\DLC_Model_Script_Video\Video\Gantry_Video_Trim_360P.mp4"
video_file_name = r"LIVE_VIDEO_CAM2_2023-11-03_15-06-47.mp4"
# video_file_name = r"H:\Other computers\My PC\PhD_UOM\General\BSBRL\Projects\Motorized_Commutator\Videos\Gantry_Video_Trim_480P.mp4"

# Serial conection parameters
selected_serial_port = "COM4"
baud_rate = 115200						# 115200
serial_timeout = 1
connect_arduino = True

# Camera and video
camera_index = 0
video_frame_to_skip = 0				# 10 to simulate 3 fps
camera = False
img_frames_to_read = 6 * 60 * 5 # 30*60*2			# 30*60*4  # *60*2	# 1000 | 1801 for 1 minute

	
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
