## Introduction

This is the code that is used to run on the client computer. There is a simple web interface for the user to select the operation of the robot. The user should use this web interface and not the interface on the main segmentation script.

## Setup

First, ensure that the computer is connected to the same local network as the Odroid and the Segmentation computer. WiFi will allow for wireless connection, but the speed of the stream may be slow. The client computer does not need to have a fixed IP address if there is a DHCP server on the router. *We recommend that the robot computer (i.e., Odroid) and Segmentation Computer should be on fixed IP address.*


Download and all the code and resources in the folder `frontendUI`

Next, open the file `client.html` using any text editor and edit the following parameters

1. IP address of the robot computer running rosbridge (i.e., Odroid) 
```angular2html
var ros = new ROSLIB.Ros({
      url: 'ws://192.168.0.2:9090'
    });
```
Change the url to ``ws://<ip-address>:9090`` By default, the roslib websocket port should be on 9090.

2. IP address of the video stream from Flask (i.e, Segmentation Computer)
```angular2html
  <img id="bg" src="http://192.168.0.3:5000/video_feed">
```

Change the src to ``http://<ip-address>:5000/video_feed`` The default port should be port 5000.

To check the IP address on Ubuntu, open `terminal` and type `hostname-I`.
## Running the Script

Using your browser, open the `client.html` script. 

##Requirements
Most browser should be able to run this. Tested browser includes:
- Mozilla Firefox
- Google Chrome

A picture of the client is shown below.

![demo image](./demo.JPG)