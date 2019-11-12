import cv2
import numpy as np

class Trackbar():
    def __init__(self):
        self.lo_bound = [0,0,0]
        self.hi_bound = [179,255,255]
        self.lo_val = [41,20,0]
        self.hi_val = [72,255,180]
        self.window_name = 'Parameter Adjustment'
        self.low_H = 'Low H'
        self.low_S = 'Low S'
        self.low_V = 'Low V'
        self.high_H = 'High H'
        self.high_S = 'High S'
        self.high_V = 'High V'
        cv2.namedWindow(self.window_name)
        cv2.createTrackbar(self.low_H, self.window_name, self.lo_bound[0], self.hi_bound[0], self.on_low_H)
        cv2.createTrackbar(self.high_H, self.window_name, self.lo_bound[0], self.hi_bound[0], self.on_high_H)
        cv2.createTrackbar(self.low_S, self.window_name, self.lo_bound[1], self.hi_bound[1], self.on_low_S)
        cv2.createTrackbar(self.high_S, self.window_name, self.lo_bound[1], self.hi_bound[1], self.on_high_S)
        cv2.createTrackbar(self.low_V, self.window_name, self.lo_bound[2], self.hi_bound[2], self.on_low_V)
        cv2.createTrackbar(self.high_V, self.window_name, self.lo_bound[2], self.hi_bound[2], self.on_high_V)

    def __del__(self):
        cv2.destroyWindow(self.window_name)

    def on_low_H(self,val):
        self.lo_val[0] = val
        self.lo_val[0] = min(self.hi_val[0]-1, self.lo_val[0])
        cv2.setTrackbarPos(self.low_H, self.window_name, self.lo_val[0])
    def on_high_H(self,val):
        self.hi_val[0] = val
        self.hi_val[0] = max(self.hi_val[0], self.lo_val[0]+1)
        cv2.setTrackbarPos(self.high_H, self.window_name, self.hi_val[0])
    def on_low_S(self,val):
        self.lo_val[1] = val
        self.lo_val[1] = min(self.hi_val[1]-1, self.lo_val[1])
        cv2.setTrackbarPos(self.low_S, self.window_name, self.lo_val[1])
    def on_high_S(self,val):
        self.hi_val[1] = val
        self.hi_val[1] = max(self.hi_val[1], self.lo_val[1]+1)
        cv2.setTrackbarPos(self.high_S, self.window_name, self.hi_val[1])
    def on_low_V(self,val):
        self.lo_val[2] = val
        self.lo_val[2] = min(self.hi_val[2]-1, self.lo_val[2])
        cv2.setTrackbarPos(self.low_V, self.window_name, self.lo_val[2])
    def on_high_V(self,val):
        self.hi_val[2] = val
        self.hi_val[2] = max(self.hi_val[2], self.lo_val[2]+1)
        cv2.setTrackbarPos(self.high_V, self.window_name, self.hi_val[2])


if __name__ == '__main__':
    img = np.zeros((300,512,3), np.uint8)
    tune = Trackbar()
    while(1):
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            del tune
            exit()
