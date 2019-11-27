import numpy as np
import cv2
import pprint
from detectUtils import *
import pprint

class DetectParking:
    def __init__(self, image):
        self.parking_color = "blue"
        self.rho = 1
        self.angle = np.pi / 180
        self.min_threshold = 10
        self.min_line_segs = 0
        # use number of top lines to compute center of stop line (must < min_line_segs)
        self.num_top_lines = 3
        self.max_flat_slope = 0.1
        self.min_vert_slope = 0.6
        self.stop_x_coord = -1
        self.stop_y_coord = -1
        self.img = image
        self.vert_pos_lines = []
        self.vert_neg_lines = []
        self.flat_lines = []

    def filter_parking_color(self):
        self.img = cv2.GaussianBlur(self.img, (5,5), 0)
        self.edges = color_mask(self.img, self.parking_color)
        cv2.imshow("edges",self.edges)

    def find_parking_edges(self):
        self.edges = cv2.cvtColor(self.edges, cv2.COLOR_BGR2GRAY)
        self.line_segs = cv2.HoughLinesP(self.edges, self.rho,self.angle, self.min_threshold, np.array([]), minLineLength=8, maxLineGap=4)

    @staticmethod
    def abs_slope(x1, y1, x2, y2, abs_val=True):
        if abs_val == False:
            return (y2-y1)/(x2-x1)
        else:
            return abs((y2-y1)/(x2-x1))

    def preprocess_lines(self):
        if self.line_segs is not None:
            self.line_segs = [l for l in self.line_segs if l[0][0] != l[0][2]]

    def extract_flat_lines(self):
        if self.line_segs is not None:
            self.flat_lines = [l for l in self.line_segs if DetectParking.abs_slope(l[0][0], l[0][1], l[0][2], l[0][3]) < self.max_flat_slope]

    def extract_vert_lines(self):
        if self.line_segs is not None:
            self.vert_pos_lines = [l for l in self.line_segs if DetectParking.abs_slope(l[0][0], l[0][1], l[0][2], l[0][3], False) > self.min_vert_slope]
            self.vert_neg_lines = [l for l in self.line_segs if DetectParking.abs_slope(l[0][0], l[0][1], l[0][2], l[0][3], False) < -1 * self.min_vert_slope]
            #DetectParking.print_lines(self.vert_pos_lines)
            #DetectParking.print_lines(self.vert_pos_lines)

    @staticmethod
    def print_lines(lines):
        print("# lines = ",(len(lines)) if lines is not None else 0)
        pprint.pprint(lines)

    def compute_stop_x_coord(self):
        if self.vert_pos_lines is not None and len(self.vert_pos_lines) > self.min_line_segs and \
        self.vert_neg_lines is not None and len(self.vert_neg_lines) > self.min_line_segs:
            self.vert_pos_line = np.asarray(self.vert_pos_lines).mean(axis=0)
            self.vert_neg_line = np.asarray(self.vert_neg_lines).mean(axis=0)
            #DetectParking.print_lines(self.flat_line)
            pos_x_coord = (self.vert_pos_line[0][0] + self.vert_pos_line[0][2]) / 2
            neg_x_coord = (self.vert_neg_line[0][0] + self.vert_neg_line[0][2]) / 2
            self.stop_x_coord = int((pos_x_coord + neg_x_coord) / 2)

        else:
            self.stop_y_coord = -1

    def compute_stop_y_coord(self):
        if self.flat_lines is not None and len(self.flat_lines) > self.min_line_segs:
            self.flat_line = np.asarray(self.flat_lines).mean(axis=0)
            #DetectParking.print_lines(self.flat_line)
            self.stop_y_coord = int((self.flat_line[0][1] + self.flat_line[0][3]) / 2)

        else:
            self.stop_y_coord = -1

    def run(self):
        self.filter_parking_color()
        self.find_parking_edges()
        self.preprocess_lines()
        self.extract_flat_lines()
        self.extract_vert_lines()
        self.compute_stop_x_coord()
        self.compute_stop_y_coord()
        print('stop (x, y) = ', self.stop_x_coord, self.stop_y_coord)
        return (self.stop_x_coord, self.stop_y_coord)


def main():
    park = DetectParking(None)
    park.run()

if __name__ == "__main__":
    main()
