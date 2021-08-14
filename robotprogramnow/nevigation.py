import time
import cv2
import numpy as np
import rospy
import torch

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16

import pointselectionscript
from roscomm import *
from mmseg.apis import inference_segmentor, init_segmentor
from flask import Flask, render_template, Response
from threading import Thread
import PySimpleGUI as sg
import os.path
from calibratecam import calibratecam

camnum = 2  # check this and edit the camera that is used for the segmentation
moveflag = 0  # master moveflag button as deadman switch
config = "/home"  # default config pointer
checkpoint = "/home"  # defualt config pointer

dpfps = 0

# robotmove variables
roll = pitch = yaw = 0.0
x = y = z = w = 0.0
initial_heading = 0.0
direction = "stop"
f_angle = 0.0

app = Flask(__name__)


@app.route('/')
def index():
    return render_template('index.html')


def gen():
    while True:
        ret, jpeg = cv2.imencode('.jpg', segframe)
        test = jpeg.tobytes()
        frame = test
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def rossubscribe():
    rospy.Subscriber('/joy', Joy, controllercallback, queue_size=1)
    rospy.Subscriber('/odom', Odometry, odometryCb)
    rospy.Subscriber('/positionx', Int16, positionxCb)
    rospy.Subscriber('/positiony', Int16, positionyCb)


def robotturn():
    global autonomousmode, moveflag, initial_heading, f_angle, distance, direction, posex, rate, percent_undrivable, dpfps
    autonomousmode = False

    rossubscribe()
    initial_heading = yaw
    turnnumber = 1

    # FPS init
    tic = time.perf_counter()
    count = 0

    while not rospy.is_shutdown():
        segmentation()

        # FPS Calculation
        count = count + 1
        toc = time.perf_counter()
        if count == 1:
            t = toc - tic
            fps = 1 / t
            dpfps = "{:.2f}".format(fps)
            print("fps: " + dpfps)
            count = 0
            tic = time.perf_counter()

        # CV2 window waitkey
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
    global autonomousmode, moveflag, distance, posex, percent_undrivable, dpfps, posex, posey, x11, x12

    rossubscribe()
    x11 = posex
    y11 = posey
    currentdist = 0
    # FPS init
    tic = time.perf_counter()
    count = 0

    while not rospy.is_shutdown():
        segmentation()

        # FPS Calculation
        count = count + 1
        toc = time.perf_counter()
        if count == 1:
            t = toc - tic
            fps = 1 / t
            dpfps = "{:.2f}".format(fps)
            print("fps: " + dpfps)
            count = 0
            tic = time.perf_counter()

        ch = cv2.waitKey(1)
        if ch == 27 or ch == ord('q') or ch == ord('Q'):
            break

        rossubscribe()

        if moveflag == 0 and autonomousmode:
            stop()

        if (moveflag == 1) and autonomousmode:

            x12 = posex
            y12 = posey
            currentdist = math.hypot(x12 - x11, y12 - y11)

            # move forward
            if currentdist < distance:
                print("currentdistance", currentdist)
                print("distance", distance)
                moveforward()
                rossubscribe()

                if percent_undrivable > 5:
                    print("stop")
                    stop()
                    autonomousmode = False
                    break

            elif currentdist >= distance:
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

    # creating CV2 window
    cv2.namedWindow('video', cv2.WINDOW_NORMAL)
    cv2.namedWindow('cropvideo', cv2.WINDOW_NORMAL)

    # parameters for the crop
    y1 = 600
    y2 = 720
    x1 = 300
    x2 = 980


def segmentation():
    global camera, model, device, percent_undrivable, feedtosend, segframe, dpfps

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
    overlaytext = f'cropwidth: {cropwidth} cropheight: {cropheight} fps: {dpfps}'
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
    global posex, orientationz, roll, pitch, yaw, x, y, z, w, posey

    posex = msg.pose.pose.position.x
    posey = msg.pose.pose.position.y
    orientationz = msg.pose.pose.orientation.z

    orientation_q = msg.pose.pose.orientation

    (roll, pitch, yaw) = euler_from_quaternion(orientation_q.x, orientation_q.x, orientation_q.z, orientation_q.w)
    yaw = 180 + math.degrees(yaw)


def positionxCb(data):  # method to run when posX is given
    global selectedX, flagx
    flagx = True
    selectedX = data.data


def positionyCb(data):  # method to run when posX is given
    global selectedY, flagy
    flagy = True
    selectedY = data.data


def pointselection(camera, model):
    global f_angle, c, direction, exitflag, segframe, flagy, flagx, selectedX, selectedY

    flagx = False
    flagy = False

    while True:
        ret_val, img = camera.read()
        result = inference_segmentor(model, img)
        maskframe, segframe = model.show_result(img, result, wait_time=1, onlymask=True)  # save model result as frame
        if flagx and flagy:
            pointselectionscript.on_EVENT_CLICK(selectedX, selectedY)
        cv2.imshow('video', segframe)

        ch = cv2.waitKey(1)
        if ch == 27 or ch == ord('q') or ch == ord('Q') or pointselectionscript.exitflag:
            pointselectionscript.exitflag = False
            break

    pointselectionscript.c = pointselectionscript.c / 100
    return pointselectionscript.f_angle, pointselectionscript.c, pointselectionscript.direction


def startnev(pconfig, pcheckpoint):
    global config, checkpoint, f_angle, distance, direction, rate, camera, model, feedtosend
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
            ch = cv2.waitKey(1)
            if ch == 27 or ch == ord('q') or ch == ord('Q'):
                break

    except rospy.ROSInterruptException:
        pass


def UIinit():
    global window
    SelectionColumn = [
        [
            sg.Text("Config File"),
            sg.In(size=(25, 1), enable_events=True, key="-CONFIGFOLDER-"),
            sg.FileBrowse()

        ],

        [
            sg.Text("Checkpoint File"),
            sg.In(size=(25, 1), enable_events=True, key="-CHECKPOINTFOLDER-"),
            sg.FileBrowse()
        ]

    ]

    ButtonColumn = [
        [
            sg.Button(button_text="Start", key="-Start-", size=(10, 1), enable_events=True),
            sg.Button(button_text="Stop", key="-Stop-", size=(10, 1), enable_events=True),
            sg.Button(button_text="Calibrate", key="-Calibrate-", size=(10, 1), enable_events=True),
        ]
    ]

    CropFrame = [
        [
            sg.Text(text="1280*720", size=(10, 1), key="-CropSize-", enable_events=True),
        ]
    ]

    PixelFrame = [
        [
            sg.Text(text="921600", size=(10, 1), key="-Pixel-", enable_events=True),
        ]
    ]

    layout = [
        [
            [sg.Column(SelectionColumn)],
            [sg.Column(ButtonColumn)],
            [sg.Frame(title="CropSize", layout=CropFrame, element_justification="center"),
             sg.Frame(title="Pixel Count", layout=PixelFrame, element_justification="center")],
        ]
    ]

    window = sg.Window("GUI", layout, element_justification="center")


if __name__ == '__main__':

    UIinit()
    # initialise app to display on website
    flaskThread = Thread(target=app.run, kwargs={'host': "192.168.0.3"})

    while True:

        event, values = window.read()

        if event == sg.WIN_CLOSED:
            break

        if event == "-Start-":
            Config_files = values["-CONFIGFOLDER-"]
            Checkpoint_files = values["-CHECKPOINTFOLDER-"]
            print(Config_files, "\n", Checkpoint_files)
            flaskThread.start()

            startnev(Config_files, Checkpoint_files)

        if event == "-Stop-":
            break

        if event == "-Calibrate-":
            calibratecam()

    window.close()
