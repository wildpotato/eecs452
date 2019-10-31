import cv2
import time
import numpy as np
import matplotlib.pyplot as plt

input_file = "green_lines.jpg"
masked_file = "masked.jpg"
output_file = "result.jpg"

def color_mask(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Green color
    low_green = np.array([25, 52, 72])
    high_green = np.array([102, 255, 255])
    green_mask = cv2.inRange(hsv_img, low_green, high_green)
    green = cv2.bitwise_and(img, img, mask=green_mask)
    cv2.imwrite(masked_file, green)

def detectEdge(frame):
    # img = cv2.imread(frame) # after_transform.jpg
    img = frame
    color_mask(img)
    time.sleep(0.3)
    mask_img = cv2.imread(masked_file)
    #gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    #edges = cv2.Canny(gray,50,150,apertureSize = 3)
    edges = cv2.Canny(mask_img,50,150,apertureSize = 3)

    lines = cv2.HoughLines(edges,1,np.pi/180,150)
    max_y = 0
    if lines is None:
        return 0, mask_img
    
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
        cv2.line(mask_img,(x1,y1),(x2,y2),(0,0,255),2)

    # print("closest line index = ", max_y)
    # cv2.imwrite(output_file,mask_img)
    
    return max_y,mask_img
    
def main():
    cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
    # cap.set(cv2.CAP_PROP_FPS,10)
    
    time.sleep(0.1)

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
    
        if cv2.waitKey(1) == ord('q'):
            break
        # max_line,frame = detectEdge(frame)
        
        cv2.imshow('frame', frame)
        
    cap.release()

if __name__ == "__main__":
    main()
