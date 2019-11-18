import cv2
import math
import numpy as np
import logging
from trackbar import Trackbar

IMAGE_DEBUG = True

#######################
### color filtering ###
#######################
def tune_color_mask(img,color,tune):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    low = np.array(tune.lo_val)
    high = np.array(tune.hi_val)
    # high, low = get_color_params(color)
    img_mask = cv2.inRange(img, low, high) #adjust
    img = cv2.bitwise_and(img, img, mask=img_mask)
    cv2.imshow('Filtered', img)
    return img

# dictionary with {color, {H_hi, H_lo, S_hi, S_lo, V_hi, V_lo}}
color_tape = {
"yellow": [33, 16, 217, 130, 203, 129],
"orange": [26, 0, 237, 134, 110, 204],
"pink": [176, 166, 150, 99, 219, 154],
"red": [179, 168, 232, 134, 196, 103],
"blue": [138, 86, 200, 115, 213, 118],
"green": [72, 40, 255, 20, 180, 0],
}

def get_color_params(color):
    if color not in color_tape.keys():
        print("color not recognized!")
        return None
    high = np.array([color_tape[color][0], color_tape[color][2], color_tape[color][4]])
    low = np.array([color_tape[color][1], color_tape[color][3], color_tape[color][5]])
    return high, low

def color_mask(img,color):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #low = np.array([low_H, low_S, low_V])
    #high = np.array([high_H, high_S, high_V])
    high, low = get_color_params(color)
    img_mask = cv2.inRange(img, low, high) #adjust
    img = cv2.bitwise_and(img, img, mask=img_mask)
    if IMAGE_DEBUG:
        cv2.imshow('Filtered', img)
    return img

########################
### helper functions ###
########################
def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)
    # only focus bottom half of the screen
    polygon = np.array([[
        (0, height * 1 / 2),
        (width, height * 1 / 2),
        (width, height),
        (0, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges

def detect_lines(img,color):
    ## filter out edges
    mask = color_mask(img, color)
    edges = cv2.Canny(mask, 200, 400)
    ## crop edges
    cropped_edges = region_of_interest(edges)
    ## find lines
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold,\
                                    np.array([]), minLineLength=8, maxLineGap=4)
    return line_segments

def make_points(height, width, line):
    # height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 0.5)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / float(slope))))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / float(slope))))
    return [[x1, y1, x2, y2]]

def combine_lines(height, width, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines

    # height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_bound = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_bound = width * boundary # right lane line segment should be on left 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_bound and x2 < left_bound:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_bound and x2 > right_bound:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(height, width, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        #OverflowError: cannot convert float infinity to integer
        lane_lines.append(make_points(height, width, right_fit_average))

    logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]
    return lane_lines


##################
### main logic ###
##################

def detect_lanes(frame):
    h, w, _ = frame.shape
    lines = detect_lines(frame,'green')
    lane_lines = combine_lines(h,w,lines)
    return lane_lines

def compute_current_angle(frame, lane_lines):
    """
    Find the current angle based on lane line coordinate
    We assume that camera is calibrated to point to dead center
    """
    if len(lane_lines) == 0:
        logging.info('No lane lines detected, do nothing')
        return -90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        logging.debug('Only detected one lane line, just follow it. %s' % lane_lines[0])
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = int(width / 2.0 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    y_offset = int(height / 2)
    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    cur_angle = -angle_to_mid_deg + 90 # this is current angle of the front wheel

    logging.debug('new current angle: %s' % cur_angle)
    return cur_angle

#########################
### display functions ###
#########################

def display_lanes(frame, lines, lane_color=(0, 255, 0), lane_width=5):
    lane_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(lane_image, (x1, y1), (x2, y2), lane_color, lane_width)
    lane_image = cv2.addWeighted(frame, 0.8, lane_image, 1, 1)
    return lane_image

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=3):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape
    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image
