# Skeleton code to be filled in by students for EECS 452 ball tracking lab
# Authors:
#   Ben Simpson
#   Siddharth Venkatesan

###############
### INCLUDE ###
###############
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
from matplotlib import pyplot as plt
import numpy as np
from timer import Timer


max_value = 255
max_value_H = 360//2
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

########################
### HELPER FUNCTIONS ###
########################

# Function for performing color subtraction
# Inputs:
#   img     Image to have a color subtracted.
#   color   Color to be subtracted.  Integer ranging from 0-255
# Output:
#   Grayscale image of the size as img with higher intensity denoting greater color difference
def color_subtract(img, color):

    #TODO: Convert img to HSV format
    img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    #TODO: Extract only h-values
    h_vals = img[:,:,0]
    h_vals = h_vals.astype('int16')
    #TODO: Take the difference of each pixel and the chosen H-value
    h_diff = h_vals - color
    #TODO: Take absolute value of the pixels
    h_diff = np.absolute(h_diff)
    h_diff = h_diff.astype('uint8')
    final_img = h_diff

    return final_img

# Function to find the centroid and radius of a detected region
# Inputs:
#   img         Binary image
#   USE_IDX_IMG Binary flag. If true, use the index image in calculations; if
#               false, use a double nested for loop.
# Outputs:
#   center      2-tuple denoting centroid
#   radius      Radius of found circle
def identify_ball(img, USE_IDX_IMG):
    # Find centroid and number of pixels considered valid
    w, h = img.shape
    center, radius = (0,0), 0

    # Double-nested for loop code
    if USE_IDX_IMG == False:
        k = 0
        x_sum = 0
        y_sum = 0

        # TODO: Calculate x_sum, y_sum, and k
        for y in range(h):
            for x in range(w):
                if img[x,y] != 0:
                    k+=1
                    x_sum+=x
                    y_sum+=y


        # TODO: Calculate the center and radius using x_sum, y_sum, and k.
        if(k != 0):
            radius = int(np.sqrt(k/np.pi))
            center = (int(x_sum/k), int(y_sum/k))
        else:
            # TODO: Don't forget to account for the boundary condition where k = 0.
            pass

    # Use index image
    else:
        # Calculate number of orange pixels
        k = np.sum(img)

        if k != 0:
            # TODO: Calculate the center and radius using the index image vectors
            #       and numpy commands

            # Index image vectors
            x_idx = np.expand_dims(np.arange(w),1)
            y_idx = np.expand_dims(np.arange(h),0)

            new_x = np.dot(np.transpose(x_idx), img)
            new_y = np.dot(img, np.transpose(y_idx))
            cx = np.sum(new_x)
            cy = np.sum(new_y)

            center = (int(cx/k), int(cy/k))
            radius = int(np.sqrt(k/(np.pi*255)))


    return center, radius

# Function to find the centroid and radius of a detected region using OpenCV's
# built-in functions.
# Inputs:
#   img         Binary image
# Outputs:
#   center      2-tuple denoting centroid
#   radius      Radius of found circle
def contours_localization(img):

    # TODO: Use OpenCV's findContours function to identify contours in img.
    #       Assume the biggest contour is the ball and determine its center and
    #       radius. Do not forget to deal with the boundary case where no
    #       contours are found.
    center, radius = (0,0), 0

    contours, hierarchy = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # k = max(contours, key = cv2.contourArea)
        cnt = contours[0] # max contour?
        M = cv.moments(cnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        k = cv.contourArea(cnt)

        radius = int(np.sqrt(k/np.pi))
        center = (int(cx/k), int(cy/k))

    return center, radius

################################
### CONSTANTS AND PARAMETERS ###
################################
# Flag for indicating how we perform color thresholding
USE_DIFFERENCE_IMAGE = True
# Flag for indicating how we perform ball localization
USE_LOCALIZATION_HEURISTIC = True
# Flag to indicate computation method for localization heuristic
USE_IDX_IMG = True

# Flag for indicating if we want the function timers to print every frame
FTP = True

# Values for color thresholding (default trackbar values)
# TODO: Find and tune these. Uncomment lines below and assign values to variables.
H_val = 9  # H-value of ball for difference image
thold_val = 0  # Threshold value for difference image

######################
### INITIALIZATION ###
######################

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)

#################
### TRACKBARS ###
#################

# Empty callback function for slider updates
def nothing(x):
    pass

# TODO: Make trackbar window
window = np.zeros((300,512,3), np.uint8)
cv2.namedWindow(window_parameters_name)
# TODO: Create trackbars. Use variables you defined in the CONSTANTS AND
#       PARAMETERS section to initialize the trackbars. (Section 3.4)
cv2.createTrackbar('Threshold',window_parameters_name,0,255,nothing)
cv2.createTrackbar(low_H_name, window_parameters_name, low_H, max_value_H, on_low_H_thresh_trackbar)
cv2.createTrackbar(high_H_name, window_parameters_name, high_H, max_value_H, on_high_H_thresh_trackbar)
cv2.createTrackbar(low_S_name, window_parameters_name, low_S, max_value, on_low_S_thresh_trackbar)
cv2.createTrackbar(high_S_name, window_parameters_name, high_S, max_value, on_high_S_thresh_trackbar)
cv2.createTrackbar(low_V_name, window_parameters_name, low_V, max_value, on_low_V_thresh_trackbar)
cv2.createTrackbar(high_V_name, window_parameters_name, high_V, max_value, on_high_V_thresh_trackbar)

cv2.setTrackbarPos('Threshold',window_parameters_name, 4)

##############
### TIMERS ###
##############

color_threshold_timer = Timer(desc="  color threshold",printflag=FTP)
contours_timer = Timer(desc="  contours",printflag=FTP)
img_disp_timer = Timer(desc="  display images",printflag=FTP)
# TODO: Add timers for difference image calculation, box filtering, and thresholding (Section 4)
diff_timer = Timer(desc="  color difference",printflag=FTP)
filter_timer = Timer(desc="  box filter",printflag=FTP)
threshold_timer = Timer(desc="  threshold function",printflag=FTP)

#################
### MAIN LOOP ###
#################

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    frame_start = time.time() # Start timer for whole frame

    #######################
    ### READ INPUT DATA ###
    #######################
    # TODO: Read trackbar positions (Section 3.4)
    # H_val = cv2.getTrackbarPos('H value',window_parameters_name)
    thold_val = cv2.getTrackbarPos('Threshold',window_parameters_name)
    # Grab raw NumPy array representing the image
    image = frame.array

    ##########################
    ### COLOR THRESHOLDING ###
    ##########################
    color_threshold_timer.start_time()
    # Use difference image method
    if USE_DIFFERENCE_IMAGE == True:
        # Calculate difference image
        diff_timer.start_time()
        # TODO: 1) Generate difference image using color subtract function (Section 3.1)
        img_diff = color_subtract(image,H_val)
        # TODO: 2) Time function (Section 4)
        diff_timer.end_time()

        # Box filter
        filter_timer.start_time()
        # TODO: 1) Implement box filter (Section 3.3)
        img_diff = cv2.boxFilter(img_diff,-1,(5,5))
        # TODO: 2) Expand dimensions of the resulting array (Section 3.3)
        # TODO: 3) Time function (Section 4)
        filter_timer.end_time()

        # Threshold
        threshold_timer.start_time()
        # TODO: 1) Threshold (Section 3.2)
        ret,thresh = cv2.threshold(img_diff,thold_val,255,cv2.THRESH_BINARY_INV)
        # TODO: 2) Time function (Section 4)
        threshold_timer.end_time()

    # Use HSV range method
    else:
        # TODO: Perform HSV thresholding
        frame_HSV = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        thresh = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))


    color_threshold_timer.end_time()

    #########################
    ### BALL LOCALIZATION ###
    #########################
    contours_timer.start_time()
    if USE_LOCALIZATION_HEURISTIC == True:
        # Use the heuristic
        center, radius = identify_ball(thresh, USE_IDX_IMG)
    else:
        # Use OpenCV's findContours function
        center, radius = contours_localization(thresh)

    # Draw the circle
    cv2.circle(image,center, radius,(0,255,0),2)
    contours_timer.end_time()

    ####################################################
    ### DISPLAY THE IMAGE, DEAL WITH KEYPRESSES, ETC ###
    ####################################################
    # Show the image
    img_disp_timer.start_time()
    cv2.imshow("Frame", image)
    # TODO: Show the difference frame (Section 3.1)
    cv2.imshow("Color Difference", img_diff)
    # TODO: Show the binary frame (Section 3.2)
    cv2.imshow("Binary",thresh)

    img_disp_timer.end_time()

    # Get keypress
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

    # TODO: if the 'c' key is pressed, capture and display the camera image
    # using pyplot
    elif key == ord("c"):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        plt.imshow(image)
        plt.show()


    # Show time to process whole frame
    frame_end = time.time()
    elapsed_time = frame_end-frame_start
    print("Frame processed in %.04f s (%02.2f frames per second)"%(elapsed_time, 1.0/elapsed_time))

# Close all windows
cv2.destroyAllWindows()
