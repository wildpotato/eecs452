import numpy as np
import cv2 as cv
import time

cap = cv.VideoCapture(0)

cap.set(cv.CAP_PROP_FRAME_WIDTH,640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT,480)

time.sleep(0.1)

if not cap.isOpened():
    print("Cannot open camera")
    exit()


while True:

    ret, frame = cap.read()

    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    cv.imshow('frame', gray)
    if cv.waitKey(1) == ord('q'):
        break



cap.release()
cv.destroyAllWindows()
