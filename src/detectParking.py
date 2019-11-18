import numpy as np
import cv2
import pprint
from detectUtils import *


class DetectParking:
    def __init__(self, image):
        self.parking_color = "blue"
        self.stop_coord = None
        self.parking_center = None
        self.rho = 1
        self.angle = np.pi / 180
        self.min_threshold = 10
        self.min_line_segs = 20
        # use number of top lines to compute center of stop line (must < min_line_segs)
        self.num_top_lines = 3
        self.img = image

    def filter_parking_color(self):
        self.edges = color_mask(self.img, self.parking_color)
        #cv2.imshow("edges",self.edges)
        
    def find_parking_edges(self):
        self.edges = cv2.cvtColor(self.edges, cv2.COLOR_BGR2GRAY)
        self.line_segs = cv2.HoughLinesP(self.edges, self.rho,self.angle, self.min_threshold, np.array([]), minLineLength=8, maxLineGap=4)

    def print_line_segs(self):
        print((len(self.line_segs)) if self.line_segs is not None else None)

    def compute_parking_center(self):
        if self.line_segs is not None and len(self.line_segs) > self.min_line_segs:
            avg = self.line_segs.mean(axis=0)
            pprint.pprint(avg)
            self.parking_center = ((avg[0][0] + avg[0][2]) / 2, (avg[0][1] + avg[0][3]) / 2)
            print("parking center = ", self.parking_center)

    def compute_stop_coord(self):
        if self.line_segs is not None and len(self.line_segs) > self.min_line_segs:
            self.sample_lines = np.squeeze(self.line_segs)
            self.sample_lines = self.sample_lines[self.sample_lines[:,1].argsort()]
            self.sample_lines = self.sample_lines[0:self.num_top_lines]
            x_coord = (self.sample_lines[:,0].mean() + self.sample_lines[:,2].mean()) / 2
            y_coord = (self.sample_lines[:,1].mean() + self.sample_lines[:,3].mean()) / 2
            print("stop coords= ", x_coord, y_coord)

    def run(self):
        self.filter_parking_color()
        self.find_parking_edges()
        self.print_line_segs()
        #self.compute_parking_center()
        self.compute_stop_coord()

def main():
    park = DetectParking(None)
    park.run()

if __name__ == "__main__":
    main()
