import cv2  # import the OpenCV library
import numpy as np  # import the numpy
import math
from mmseg.apis import inference_segmentor, init_segmentor
import torch

robo_x = 337.5
robo_y = 0

xcord = 0
ycord = 0

f_angle = 0.0

exitflag = False


def homography(x, y):
    # provide points from image 1
    pts_src = np.array([[0, 720], [0, 170], [1277, 171], [1277, 717]])
    # corresponding points from image 2 (i.e. (154, 174) matches (212, 80))
    pts_dst = np.array([[293, 56.1], [0, 480], [645, 480], [380, 56.1]])

    # calculate matrix H
    h, status = cv2.findHomography(pts_src, pts_dst)

    # provide a point you wish to map from image 1 to image 2
    a = np.array([[x, y]], dtype='float32')
    a = np.array([a])

    # finally, get the mapping
    pointsOut = cv2.perspectiveTransform(a, h)
    print(pointsOut)
    return pointsOut


def calculate(xcords, ycords):
    global f_angle, c, direction

    a = xcords - robo_x
    b = ycords - robo_y

    c = math.sqrt((a * a) + (b * b))

    angle = math.atan2(a, (b + 52))

    if angle < 0:
        angle = abs(angle)
        direction = "left"
    else:
        direction = "right"

    f_angle = angle * 180 / math.pi
    f_angle = f_angle - f_angle * 0.155555556

    print("Angle is :", f_angle)
    print("Distance is :", c)
    print("Direction is :", direction)


def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    global exitflag
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        coords = homography(x, y)

        calculate(coords[0][0][0], coords[0][0][1])


        exitflag = True


def pointselection(camera, model):
    global f_angle, c, direction, exitflag

    while True:
        print("Doing this")
        ret_val, img = camera.read()
        result = inference_segmentor(model, img)
        maskframe, segframe = model.show_result(img, result, wait_time=1, onlymask=True)  # save model result as frame

        cv2.setMouseCallback("video", on_EVENT_LBUTTONDOWN)
        cv2.imshow('video', segframe)

        key = cv2.waitKey(1)
        if key == 27 or exitflag:
            exitflag = False
            break
    c = c / 100
    return f_angle, c, direction


'''
def pointselection(camnum, config, checkpoint):

    global f_angle, c, direction, exitflag

    device = torch.device('cuda:0')

    model = init_segmentor(config, checkpoint, device=device)

    camera = cv2.VideoCapture(camnum)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cv2.namedWindow('video', cv2.WINDOW_NORMAL)
    cv2.namedWindow('cropvideo', cv2.WINDOW_NORMAL)


    while True:

        ret_val, img = camera.read()
        result = inference_segmentor(model, img)
        maskframe, segframe = model.show_result(img, result, wait_time=1, onlymask=True)  # save model result as frame


        cv2.setMouseCallback("video", on_EVENT_LBUTTONDOWN)
        cv2.imshow('video', segframe)

        key = cv2.waitKey(1)
        if key == 27 or exitflag:
            break
    c = c / 100
    return f_angle, c, direction
'''
