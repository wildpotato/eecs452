"""
Simple script to resize images contained in a directory
Usage: python resize_image.py -d path_to_dir -s width height
"""
from PIL import Image
import os
import argparse
def rescale_images(directory, size):
    for img in os.listdir(directory):
        if img.endswith(".DS_Store"):
            continue
        im = Image.open(directory+img)
        im_resized = im.resize(size, Image.ANTIALIAS)
        im_resized.save(directory+img)
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Rescale images")
    parser.add_argument('-d', '--directory', type=str, required=True, help='Directory containing the images')
    parser.add_argument('-s', '--size', type=int, nargs=2, required=True, metavar=('width', 'height'), help='Image size')
    args = parser.parse_args()
    rescale_images(args.directory, args.size)
