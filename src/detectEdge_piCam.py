from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
from matplotlib import pyplot as plt
import numpy as np
from timer import Timer
import car_dir
import motor


#####################
### INITIALIZTION ###
#####################
car_dir.setup(busnum=1)
motor.setup(busnum=1)
motor.ctrl(0)
car_dir.home()
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)


################################
### CONSTANTS AND PARAMETERS ###
################################
max_value = 255
max_value_H = 179
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value
window_parameters_name = 'Parameter Adjustment'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

low_H=28
low_S=19
low_V=172
high_H=129
high_S=130
high_V=227

#################
### TRACKBARS ###
#################
def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv2.setTrackbarPos(low_H_name, window_parameters_name, low_H)
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv2.setTrackbarPos(high_H_name, window_parameters_name, high_H)
def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv2.setTrackbarPos(low_S_name, window_parameters_name, low_S)
def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv2.setTrackbarPos(high_S_name, window_parameters_name, high_S)
def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv2.setTrackbarPos(low_V_name, window_parameters_name, low_V)
def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv2.setTrackbarPos(high_V_name, window_parameters_name, high_V)

#################
### FUNCTIONS ###
#################
def color_mask(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Green color
    low_green = np.array([low_H, low_S, low_V])
    high_green = np.array([high_H, high_S, high_V])
    green_mask = cv2.inRange(hsv_img, low_green, high_green)
    green = cv2.bitwise_and(img, img, mask=green_mask)
    return green

def detectEdge(img):
    img = color_mask(img)
    #gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    #edges = cv2.Canny(gray,50,150,apertureSize = 3)
    edges = cv2.Canny(img,50,150,apertureSize = 3)

    lines = cv2.HoughLines(edges,1,np.pi/180,150)
    max_y = 0

    if lines is None:
        return max_y, img

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
        max_y = max(max_y, y)
        cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
    print("closest line index = ", max_y)
    return max_y,img

def car_control(idx):
    motor.forward()

    if idx < 470:
        motor.setSpeed(30)
    else:
        motor.setSpeed(0)

    return None

def main():

    # ~ cv2.namedWindow(window_parameters_name)
    # ~ cv2.createTrackbar(low_H_name, window_parameters_name, low_H, max_value_H, on_low_H_thresh_trackbar)
    # ~ cv2.createTrackbar(high_H_name, window_parameters_name, high_H, max_value_H, on_high_H_thresh_trackbar)
    # ~ cv2.createTrackbar(low_S_name, window_parameters_name, low_S, max_value, on_low_S_thresh_trackbar)
    # ~ cv2.createTrackbar(high_S_name, window_parameters_name, high_S, max_value, on_high_S_thresh_trackbar)
    # ~ cv2.createTrackbar(low_V_name, window_parameters_name, low_V, max_value, on_low_V_thresh_trackbar)
    # ~ cv2.createTrackbar(high_V_name, window_parameters_name, high_V, max_value, on_high_V_thresh_trackbar)

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        image = cv2.flip(image,0)

        max_line,image = detectEdge(image)
        cv2.imshow("Frame", image)

        car_control(max_line)

        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
        if key == ord("q"):
            break
        elif key == ord("c"):
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            plt.imshow(image)
            plt.show()


if __name__ == "__main__":
    main()
