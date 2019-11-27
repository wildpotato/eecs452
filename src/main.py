import sys
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
from matplotlib import pyplot as plt
import detectUtils
from detectParking import *
import logging




Stage1 = True
Stage2 = False
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
###  LOGGING  ###
#################
def setup_logger():
    # create logger
    logger = logging.getLogger("logging info")
    logger.disabled = True
    logger.setLevel(logging.CRITICAL)

    # create console handler and set level to critical
    ch = logging.StreamHandler()
    ch.setLevel(logging.CRITICAL)

    # create formatter
    formatter = logging.Formatter('%s')
    
    # add formatter to ch
    
    ch.setFormatter(formatter)
    # add ch to logger
    logger.addHandler(ch)
    return logger



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
            car.Ki += 0.005
            print("Ki: ",car.Ki)
        elif key == ord("h"):
            car.Ki -= 0.005
            print("Ki: ",car.Ki)
        elif key == ord("k"):
            car.Kd += 0.005
            print("Kd: ",car.Kd)
        elif key == ord("l"):
            car.Kd -= 0.005
            print("Kd: ",car.Kd)

def main():
    global Stage1,Stage2
    global LANE_KEEPING_ON, SEARCH_PARKING_ON, CAR_ON
    logger = setup_logger()
    logger.critical('start main')
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        start = time.time()
        image = frame.array
        # ~ image = cv2.flip(image,-1)
        if Stage1:
            if LANE_KEEPING_ON:
                lanes = detectUtils.detect_lanes(image)
                lanes_image = detectUtils.display_lanes(image, lanes)
                cur_angle = detectUtils.compute_current_angle(image, lanes)
                lanes_image = detectUtils.display_heading_line(lanes_image, cur_angle)
                #cv2.imshow("Lanes", lanes_image)

            if CAR_ON and LANE_KEEPING_ON:
                stage1_finished = car.check_stage1_status(lanes)
                if stage1_finished:
                    Stage1 = False
                    Stage2 = True
                    print("Stage 1 Ends")
                else:
                    car.tune_pid()
                    car.motor_run(35)
                    car.servo_turn(cur_angle)

        if Stage2:
            if SEARCH_PARKING_ON:
                park = DetectParking(image)
                coords = park.run()

                if car.parking(coords):
                    print('Car parked!!!')
                    Stage2 = False

        keyboard_operation()
        rawCapture.truncate(0)
        fps = 1 / (time.time() - start)
        logger.critical("fps = ", fps)

def calibrate():
    tune = detectUtils.Trackbar()
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        image = cv2.flip(image,-1)
        image = cv2.GaussianBlur(image, (5,5), 0)
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
