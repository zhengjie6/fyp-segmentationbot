# Project Title: 2021 FYP - Semantic Segmentation for Autonomous Robot
# This script is the main script for the computer running
# the segmentation system for the semi-autonomous robot
# By: Zheng Jie and Jun Yip

import cv2
import PySimpleGUI as sg
import os.path
from calibratecam import calibratecam
from nevigation import startnev

if __name__ == '__main__':

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

    FpsFrame = [
        [
            sg.Text(text="10", size=(10, 1), key="-FPS-", enable_events=True),
        ]
    ]

    PixelFrame = [
        [
            sg.Text(text="10", size=(10, 1), key="-Pixel-", enable_events=True),
        ]
    ]

    layout = [
        [
            [sg.Column(SelectionColumn)],
            [sg.Column(ButtonColumn)],
            [sg.Frame(title="FPS", layout=FpsFrame, element_justification="center"),
             sg.Frame(title="Pixel Count", layout=PixelFrame, element_justification="center")],
        ]
    ]

    window = sg.Window("GUI", layout, element_justification="center")

    while True:

        event, values = window.read()

        if event == sg.WIN_CLOSED:
            break

        if event == "-Start-":
            Config_files = values["-CONFIGFOLDER-"]
            Checkpoint_files = values["-CHECKPOINTFOLDER-"]
            print(Config_files, "\n", Checkpoint_files)
            startnev(Config_files,Checkpoint_files)
            window["-FPS-"].update("100")

        if event == "-Stop-":
            break

        if event == "-Calibrate-":
            calibratecam()

    window.close()