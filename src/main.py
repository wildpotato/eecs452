import sys
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
from matplotlib import pyplot as plt
import detectUtils
import controlUtils as Car

# True if car is connected, false for prototyping
car_ON = False


#####################
### INITIALIZTION ###
#####################
# Control
if car_ON:
    Car.setup()
# Camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)

#################
### FUNCTIONS ###
#################
def keyboard_operation():
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        if car_ON:
            Car.motor_stop()
            Car.servo_home()
        exit()
    # ~ elif key == ord("p"):
        # ~ pid.Kp += 0.1
        # ~ print("Kp: ",pid.Kp)
    # ~ elif key == ord("l"):
        # ~ pid.Kp -= 0.1
        # ~ print("Kp: ",pid.Kp)
    # ~ elif key == ord("i"):
        # ~ pid.Ki += 0.01
        # ~ print("Kp: ",pid.Ki)
    # ~ elif key == ord("j"):
        # ~ pid.Ki -= 0.01
        # ~ print("Kp: ",pid.Ki)
    # ~ elif key == ord("t"):
        # ~ pid.Kd += 0.01
        # ~ print("Kp: ",pid.Kd)
    # ~ elif key == ord("g"):
        # ~ pid.Kd -= 0.01
        # ~ print("Kp: ",pid.Kd)

def main():
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        #image = cv2.flip(image,0)

        lanes = detectUtils.detect_lanes(image)
        lanes_image = detectUtils.display_lanes(image, lanes)
        steering_angle = detectUtils.compute_steering_angle(image, lanes)
        lanes_image = detectUtils.display_heading_line(lanes_image, steering_angle)
        print("Steering Angle:",steering_angle)
        cv2.imshow("Lanes", lanes_image)

        Car.motor_run()
        Car.servo_turn(steering_angle)


        keyboard_operation()
        rawCapture.truncate(0)

def calibrate():
    tune = detectUtils.Trackbar()
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        #image = cv2.flip(image,0)
        detectUtils.tune_color_mask(image,'green',tune)

        keyboard_operation()
        rawCapture.truncate(0)


if __name__ == "__main__":
    argv = sys.argv[1:]
    if len(argv) == 0:
        main()
    elif argv[0] == 'calibrate':
        calibrate()
    else:
        print("Wrong arugument.")
