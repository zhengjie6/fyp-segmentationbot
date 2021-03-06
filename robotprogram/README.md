## Introduction


### Starting up Odroid and Robot 
1. Turn on the E-Stop Control (clockwise) and the key is required. Wait for the system to start up. The monitor will turn on with the OS ready. 
2. UID: Odroid Pwd: odroid
3. CTRL-Alt-T to open terminal
4. (Optional) Enter command `sudo date -s"16 dec 2020 14:25:00"` and replace the date and time with the current date and time. Press `enter`. The date and time is set.
5. Start the ROS Core and robot program using the command `roslaunch npbot bot.launch`. Wait for ROS to start up. The robot IP address should default to `192.168.0.2`
6. The robot should be drivable with a joystick. Press and hold `x` on the joystick to drive.


### Setting up Segmentation Computer


### Powering off the Odroid and Robot
1. Use Ctr-C to stop the terminal command (and ROS). 
2. Type `poweroff` to poweroff the Odroid.
3. Once the monitor switches off, press the E-Stop switch to complete the shutdown.

### Troubleshooting
* Symptom: ROScore is unable to start up due to IP address issues. If ROS is unable to find the ROS Core IP at 192.168.0.2, the IP address of the Odroid is wrong (it may be connected to NPWirelessX). Use the command `ifconfig` to find out the current IP address of the system. Ensure that the router and the network switch is turn on and try to run step `5` again.

* Symptom: Serial communication error displayed in terminal. If the ROS is unable to detect the motor controller, a  Reboot the system (E-Stop). If the error still persist, check the USB connection to the Odroid and KangarooX2. 

* Symptom: Robot is not moving but no error is shown. This could be due to the joystick not being detected when the system is started up. The joystick needs to be a 
`dev/input/js0` on the Odroid. A simple fix is to reboot the Ordroid. To do so, press Ctrl-C to kill the program. Then, type `reboot`. If not, try to type `/dev/input/js*` and see if js0 appears on the list.
