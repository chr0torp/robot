import cv2
from picamera2 import Picamera2
import libcamera
import time 
import threading


def start_camera():
    print("Initializing camera...")
    picam2 = Picamera2()
    print("Configuring camera...")
    frame_width = 1920 
    frame_height = 1080
    config = picam2.create_preview_configuration(main={"format": 'RGB888', "size": (frame_width, frame_height)})
    config["transform"] = libcamera.Transform(hflip=1, vflip=1)
    picam2.configure(config)
    print("Starting camera...")
    picam2.start()
    time.sleep(1)
    print("Camera started.")
    return picam2  



def show_camera_feed(picam2):
    print("Displaying camera feed in OpenCV window.")

    try:
        while True:
            frame_rgb = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            cv2.imshow("live feed", frame_bgr)

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        cv2.destroyAllWindows()



def stop_camera(picam2):
    print("Stopping camera and closing resources...")
    picam2.stop()
    cv2.destroyAllWindows()
    print("Camera stopped and resources released.")
