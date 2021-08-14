import time
from argparse import ArgumentParser
import cv2
import numpy as np
import rospy
import torch

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

from pointselection import pointselection
from roscomm import *
from mmseg.apis import inference_segmentor, init_segmentor

camnum = 2  # check this and edit the camera that is used for the segmentation
moveflag = 0  # master moveflag button as deadman switch
config = "/home"  # default config pointer
checkpoint = "/home"  # defualt config pointer

# robotmove variables
roll = pitch = yaw = 0.0
x = y = z = w = 0.0
initial_heading = 0.0
direction = "stop"
f_angle = 0.0


def rossubscribe():
    rospy.Subscriber('/joy', Joy, controllercallback, queue_size=1)
    rospy.Subscriber('/odom', Odometry, odometryCb)


def robotturn():
    global autonomousmode, moveflag, initial_heading, f_angle, distance, direction, posex, rate, percent_undrivable
    autonomousmode = False

    rossubscribe()
    initial_heading = yaw
    turnnumber = 1
    while not rospy.is_shutdown():
        segmentation()

        ch = cv2.waitKey(1)
        if ch == 27 or ch == ord('q') or ch == ord('Q'):
            break

        rossubscribe()

        if moveflag == 0 and autonomousmode:
            stop()

        if (moveflag == 1) and autonomousmode:

            # robot to turn to desired heading

            if direction == "left":
                final_heading = initial_heading + f_angle
                print("initial final heading", final_heading)
                print("current heading", yaw)

                if final_heading < 360:
                    if yaw < final_heading:
                        turnleft()
                        if percent_undrivable > 5:
                            print("stop")
                            stop()
                            autonomousmode = False
                            break
                        rossubscribe()
                        print(final_heading)
                        print(yaw)
                    else:
                        stop()
                        break

                if final_heading > 360:
                    final_heading = final_heading - 360
                    print("Calibrated", final_heading)

                    if yaw > final_heading and turnnumber == 1:
                        turnleft()
                        if percent_undrivable > 5:
                            print("stop")
                            stop()
                            autonomousmode = False
                            break
                        rossubscribe()
                        print(final_heading)
                        print(yaw)

                    if yaw < final_heading:
                        turnnumber = 2
                        turnleft()
                        if percent_undrivable > 5:
                            print("stop")
                            stop()
                            autonomousmode = False
                            break
                        rossubscribe()
                        print(final_heading)
                        print(yaw)

                    if yaw > final_heading and turnnumber == 2:
                        stop()
                        break

            elif direction == "right":
                final_heading = initial_heading - f_angle
                print("initial final heading", final_heading)
                print("current heading", yaw)
                if final_heading > 0:
                    if (yaw > final_heading):
                        turnright()
                        if percent_undrivable > 5:
                            print("stop")
                            stop()
                            autonomousmode = False
                            break
                        rossubscribe()
                        print(final_heading)
                        print(yaw)
                    else:
                        stop()
                        break

                if final_heading < 0:
                    final_heading = 360 + final_heading
                    print("calibrated", final_heading)

                    if yaw < final_heading and turnnumber == 1:
                        turnright()
                        if percent_undrivable > 5:
                            print("stop")
                            stop()
                            autonomousmode = False
                            break
                        rossubscribe()
                        print(final_heading)
                        print(yaw)

                    if yaw > final_heading:
                        turnnumber = 2
                        turnright()
                        if percent_undrivable > 5:
                            print("stop")
                            stop()
                            autonomousmode = False
                            break
                        rossubscribe()
                        print(final_heading)
                        print(yaw)

                    if yaw < final_heading and turnnumber == 2:
                        stop()
                        break

        if not autonomousmode:
            print("In driver control")


def robotforward():
    global autonomousmode, moveflag, distance, posex, percent_undrivable

    rossubscribe()
    initialpose = posex
    finalpose = posex + distance
    while not rospy.is_shutdown():
        segmentation()

        ch = cv2.waitKey(1)
        if ch == 27 or ch == ord('q') or ch == ord('Q'):
            break

        rossubscribe()

        if moveflag == 0 and autonomousmode:
            stop()

        if (moveflag == 1) and autonomousmode:

            # move forward
            if posex < finalpose:
                print("distance", distance)
                print("Final pose", finalpose)
                print("Current pose",posex)
                moveforward()
                rossubscribe()

                if percent_undrivable > 5:
                    print("stop")
                    stop()
                    autonomousmode = False
                    break

            elif posex >= finalpose:
                stop()
                autonomousmode = False

        if not autonomousmode:
            break


def segmentationinit():
    global y1, y2, x1, x2, camera, model, device
    rossubscribe()

    device = torch.device('cuda:0')

    model = init_segmentor(config, checkpoint, device=device)

    camera = cv2.VideoCapture(camnum)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print('Press "Esc", "q" or "Q" to exit.')
    tic = time.perf_counter()
    count = 0

    # creating CV2 window
    cv2.namedWindow('video', cv2.WINDOW_NORMAL)
    cv2.namedWindow('cropvideo', cv2.WINDOW_NORMAL)

    # parameters for the crop
    y1 = 600
    y2 = 720
    x1 = 300
    x2 = 980


def segmentation():
    global camera, model, device, percent_undrivable

    ret_val, img = camera.read()
    result = inference_segmentor(model, img)
    maskframe, segframe = model.show_result(img, result, wait_time=1, onlymask=True)  # save model result as frame

    crop_frame = maskframe[y1:y2, x1:x2]

    cropwidth = crop_frame.shape[1]  # count width of crop frame
    cropheight = crop_frame.shape[0]  # count height of crop frame
    total_pix = cropwidth * cropheight  # count total pixels of crop frame

    number_of_black_pix = np.sum(crop_frame == 0) - (cropwidth * cropheight) * 2
    percent_undrivable = (number_of_black_pix / total_pix) * 100
    print('Number of black pixels:', number_of_black_pix, ' Percent undrivable', percent_undrivable, end=' ')

    # overlaytext to display width and height
    overlaytext = f'cropwidth: {cropwidth} cropheight: {cropheight}'
    cv2.displayOverlay('video', overlaytext)

    segframe = np.ascontiguousarray(segframe,
                                    dtype=np.uint8)  # used to ensure that frame array in memory is stored contiguously
    segframe = cv2.rectangle(segframe, (x1, y1), (x2, y2), (0, 0, 255), 2)  # draw center box

    # cv2 show
    cv2.imshow('video', segframe)
    cv2.imshow('cropvideo', crop_frame)


def controllercallback(data):  # method to run when RB is pressed
    global moveflag
    global autonomousmode
    moveflag = data.buttons[5]  # button RB
    if data.buttons[9] == 1:  # start button
        autonomousmode = True
    if data.buttons[8] == 1:  # back button
        autonomousmode = False


def odometryCb(msg):
    global posex, orientationz, roll, pitch, yaw, x, y, z, w

    posex = msg.pose.pose.position.x
    orientationz = msg.pose.pose.orientation.z

    orientation_q = msg.pose.pose.orientation

    (roll, pitch, yaw) = euler_from_quaternion(orientation_q.x, orientation_q.x, orientation_q.z, orientation_q.w)
    yaw = 180 + math.degrees(yaw)


def startnev(pconfig, pcheckpoint):
    global config, checkpoint, f_angle, distance, direction, rate, camera, model
    config = pconfig
    checkpoint = pcheckpoint
    try:
        rospublisherinit()  # initialise the ros node and publisher
        rate = rospy.Rate(10)
        segmentationinit()
        while True:
            f_angle, distance, direction = pointselection(camera,
                                                          model)  # f_angle: float distance: float32 direction:string
            robotturn()
            robotforward()
            key = cv2.waitKey(1)
            if key == 27:
                break
    except rospy.ROSInterruptException:
        pass
