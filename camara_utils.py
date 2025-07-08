import cv2
from picamera2 import Picamera2
import libcamera
import time 


def take_picture():
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
    print("Capturing image...")
    image = picam2.capture_array()
    print("Image captured.")
    picam2.close()
    return image

def show_image(image):
    print("Displaying image in OpenCV window.")
    cv2.imshow("Captured Image", image)
    cv2.waitKey(0)  # Wait for a key press to close the window
    cv2.destroyAllWindows()
    print("Image display closed.")

def stop_image():
    cv2.destroyAllWindows()
    print("Image capture stopped and all windows closed.")


