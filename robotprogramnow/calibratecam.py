# This script is used to calibrate the camera by using
# a customised calibration tool. This is used to ensure that the
# camera angle will remain the same for each test.

import cv2

def calibratecam():
    cv2.namedWindow("Calibrate Camera", cv2.WINDOW_NORMAL)
    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # Check if the webcam is opened correctly
    if not cap.isOpened():
        raise IOError("Cannot open webcam")

    while True:
        ret, frame = cap.read()
        frame = cv2.circle(frame, (50, 500), radius=4, color=(0, 0, 255), thickness=-1)
        frame = cv2.circle(frame, (1000, 500), radius=4, color=(0, 0, 255), thickness=-1)
        frame = cv2.circle(frame, (150, 300), radius=4, color=(0, 0, 255), thickness=-1)
        frame = cv2.circle(frame, (800, 310), radius=4, color=(0, 0, 255), thickness=-1)
        cv2.imshow('Calibrate Camera', frame)

        c = cv2.waitKey(1)
        if c == 27:
            break
        if cv2.getWindowProperty('Calibrate Camera', cv2.WND_PROP_VISIBLE) < 1:
            break

    cap.release()
    cv2.destroyAllWindows()
