from PIL import Image
import os
import argparse

def resize_images(directory, size):
    for img in os.listdir(directory):
        if img.endswith(".DS_Store"):
            continue
        im = Image.open(directory + img)
        im_resized = im.resize(size, Image.ANTIALIAS)
        im_resized.save(directory + img)

def getParser():
    parser = argparse.ArgumentParser(description="")
    parser.add_argument('-d', '--directory', type=str, required=True, help='Directory containing the images')
    parser.add_argument('-s', '--size', type=int, nargs=2, required=True, metavar=('width', 'height'), help='Image size')
    return parser

def main():
    parser = getParser()

if __name__ == "__main__":
    main()
