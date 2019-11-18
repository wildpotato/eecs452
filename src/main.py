import sys
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
from matplotlib import pyplot as plt
import detectUtils
from detectParking import *


# True if car is connected, false for algorithm prototyping
LANE_KEEPING_ON = True
CAR_ON = True
SEARCH_PARKING_ON = False

#####################
### INITIALIZTION ###
#####################
# Control
if CAR_ON:
    import controlUtils
    car = controlUtils.Car()
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
        cv2.destroyAllWindows()
        if CAR_ON:
            car.motor_stop()
            car.servo_home()
        exit()
    if CAR_ON:
        if key == ord("a"):
            car.Kp += 0.1
            print("Kp: ",car.Kp)
        elif key == ord("s"):
            car.Kp -= 0.1
            print("Kp: ",car.Kp)
        elif key == ord("g"):
            car.Ki += 0.01
            print("Ki: ",car.Ki)
        elif key == ord("h"):
            car.Ki -= 0.01
            print("Ki: ",car.Ki)
        elif key == ord("k"):
            car.Kd += 0.005
            print("Kd: ",car.Kd)
        elif key == ord("l"):
            car.Kd -= 0.005
            print("Kd: ",car.Kd)

def main():
    global LANE_KEEPING_ON
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        image = cv2.flip(image,-1)

        if LANE_KEEPING_ON:
            lanes = detectUtils.detect_lanes(image)
            if lanes is None:
                LANE_KEEPING_ON = False
            lanes_image = detectUtils.display_lanes(image, lanes)
            cur_angle = detectUtils.compute_current_angle(image, lanes)
            lanes_image = detectUtils.display_heading_line(lanes_image, cur_angle)
            print("Current angle:",cur_angle)
            cv2.imshow("Lanes", lanes_image)

        if CAR_ON:
            # ~ car.tune_pid()
            car.lane_info(lanes)
            car.motor_run()
            car.servo_turn(cur_angle)

        if SEARCH_PARKING_ON:
            cv2.imshow("before parking",image)
            park = DetectParking(image)
            park.run()

        keyboard_operation()
        rawCapture.truncate(0)

def calibrate():
    tune = detectUtils.Trackbar()
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        image = cv2.flip(image,-1)
        detectUtils.tune_color_mask(image,'blue',tune)

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
