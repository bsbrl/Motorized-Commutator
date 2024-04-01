import cv2
import time

def display_camera_feed(camera_index, display_duration=3):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Camera index {camera_index} is not available.")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow(f"Camera {camera_index}", frame)
        # cv2.namedWindow(f"Camera {camera_index}", cv2.WINDOW_NORMAL)
        # cv2.imshow(f"Camera {camera_index}", frame)
        # cv2.imshow(frame)


        # Wait for 'display_duration' seconds
        if cv2.waitKey(1) & 0xFF == 27:  # Press Esc to exit
            break
        # time.sleep(display_duration)

    cap.release()
    cv2.destroyAllWindows()

def list_and_display_connected_cameras():
    for i in range(10):
        display_camera_feed(i)

if __name__ == "__main__":
    list_and_display_connected_cameras()
