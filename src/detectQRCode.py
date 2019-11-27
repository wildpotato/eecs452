"""
Simple barcode detection module

Required installs:
    zbar (brew install zbar)
    cv2
"""

from pyzbar import pyzbar
import argparse
import cv2

class DetectQRCode:
    def __init__(self, img):
        self.img = img

    def get_barcode(self):
        self.image = cv2.imread(self.img, cv2.IMREAD_COLOR)
        cv2.imshow('image', self.image)
        self.barcodes = pyzbar.decode(self.image)

    def bound_barcode(self):
        for barcode in self.barcodes:
            (x, y, w, h) = barcode.rect
            cv2.rectangle(self.image, (x,y), (x+w,y+h), (0, 0, 255), 2)
        cv2.imshow("Barcode Image", self.image)

    def run(self):
        self.get_barcode()
        self.bound_barcode()

def get_args():
    parser = argparse.ArgumentParser(description="Barcode detector")
    parser.add_argument("-i", "--image", required=True, help="path to input image")
    return parser.parse_args()

def main():
    args = get_args()
    code = DetectQRCode(args.image)
    code.run()

if __name__ == "__main__":
    main()
