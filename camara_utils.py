import cv2
from picamera2 import Picamera2
import libcamera
import time 


def take_picture():
    print("Initializing camera...")
    picam2 = Picamera2()
    frame_width = 1920 
    frame_height = 1080
    config = picam2.create_preview_configuration(main={"format": 'RGB888', "size": (frame_width, frame_height)})
    config["transform"] = libcamera.Transform(hflip=1, vflip=1)
    picam2.configure(config)
    picam2.start()
    time.sleep(1)
    image = picam2.capture_array()
    picam2.close()
    return image

def show_image(image):
    print("Displaying image in OpenCV window.")
    cv2.imshow("Captured Image", image)
    cv2.waitKey(0)  
    cv2.destroyAllWindows()
    print("Image display closed.")

def stop_image():
    cv2.destroyAllWindows()
    print("Image capture stopped and all windows closed.")

def save_image(image, filename='captured_image.jpg'):
    print(f"Saving image to {filename}...")
    cv2.imwrite(filename, image)
    print("Image saved successfully.")


