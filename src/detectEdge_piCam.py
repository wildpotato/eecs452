from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
from timer import Timer
from matplotlib import pyplot as plt
import numpy as np
import car_dir
import motor

import trackbar

#####################
### INITIALIZTION ###
#####################
# Control
car_dir.setup(busnum=1)
motor.setup(busnum=1)
motor.ctrl(0)
car_dir.home()
# Camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)
#trackbar
# ~ trackbar.setup()

################################
### CONSTANTS AND PARAMETERS ###
################################



#################
### FUNCTIONS ###
#################
def keyboard_operation(image):
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        motor.stop()
        car_dir.home()
        exit()
    elif key == ord("c"):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        plt.imshow(image)
        plt.show()
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

def color_mask(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Green color
    low_green = np.array([trackbar.low_H, trackbar.low_S, trackbar.low_V])
    high_green = np.array([trackbar.high_H, trackbar.high_S, trackbar.high_V])
    green_mask = cv2.inRange(hsv_img, low_green, high_green) #adjust
    green = cv2.bitwise_and(img, img, mask=green_mask) # necessary?
    return green

def detectEdge(img):
    global max_deg
    img = color_mask(img)
    edges = cv2.Canny(img,50,150,apertureSize = 3) # adjust

    lines = cv2.HoughLines(edges,1,np.pi/180,150)
    max_y = 0
    max_deg = 0
    if lines is None:
        return img, max_y, max_deg

    for l in lines:
        for rho,theta in l:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            y = y1 if y1 > y2 else y2
            # ~ print("Current line degree: ",((np.pi/2)-theta)*(180/np.pi))
            line_deg = ((np.pi/2)-theta)*(180/np.pi)

        if abs(line_deg) > abs(max_deg):
            max_deg =  line_deg
        else:
            max_deg = max_deg
        max_y = max(max_y, y)
        cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
    print("closest line index = ", max_y)
    return img, max_y, max_deg

def car_control(idx, deg):
    if idx < 400 or idx == 400:
        speed = -0.05*idx+35
    elif idx > 400 and idx < 470:
        speed = 15
    else:
        speed = 0

    motor.forwardWithSpeed(int(speed))

    car_dir.turn(int(deg))



def main():

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        image = cv2.flip(image,0)

        # ~ cv2.imshow("Frame", image)
        image, max_line, max_deg = detectEdge(image)
        cv2.imshow("Filtered", image)

        car_control(max_line,max_deg)

        keyboard_operation(image)
        rawCapture.truncate(0)


if __name__ == "__main__":
    main()
