# 1 Initial Setup
This project utilizes a Raspberry Pi 4 running ROS Melodic on Raspberry Pi OS. See [Section 1.1](https://github.com/cvkittler/Metamorphic-Manufacturing/tree/EOAT_ros#11-fresh-os-and-ros-install) to setup a brand new Raspberry Pi, or [Section 1.2](https://github.com/cvkittler/Metamorphic-Manufacturing/tree/EOAT_ros#12-use-the-existing-raspberry-pi-setup-by-the-2021-2022-team) to use the existing pi on the EOAT

## 1.1 Fresh OS and ROS Install
1. Prerequisites
   1. Another computer running Ubuntu that can generate a wifi hotspot
   2. A Monitor + Display Cable (and likely a dongle to connect to the Pi's display outputs)
   3. Keyboard (Mouse not required, but nice to have)
   5. Internet Connection (You can do whatever ITS suggests for connecting to WPI-Wireless, but we just used hotspots from our laptops)

2. Setup your Raspberry Pi 4 (Pi 3 worked as well, but was slower) with Raspberry Pi OS and ROS Melodic
   1. We used a [Prebuilt Raspberry Pi Image](http://sar-lab.net/teaching/ros-images-for-raspberry-pi/) - Specifically the ROS Melodic on Raspberry Pi OS (Aug 2020) version
   2. Enable the GUI
   3. Setup [VNC Viewer](https://www.realvnc.com/en/connect/download/viewer/) or other remote desktop software of your choice

3. Install the requisite libraries (see [Section 4.2](https://github.com/cvkittler/Metamorphic-Manufacturing/tree/EOAT_ros#42-libraries-used))

4. Clone THIS BRANCH (EOAT_ros) such that the package is in ~/catkin_ws/src/ (ie you get ~/catkin_ws/src/mmmqp_eoat/src/EOAT_c.cpp etc...)

5. Setup a [Wifi Hotspot](https://www.fosslinux.com/2576/how-to-create-and-configure-wi-fi-hotspot-in-ubuntu-17-10.htm) on the Linux computer you wish to run the High Level Program from and connect the Pi to it
   1. At this point you can remote desktop in from either the computer running the hotspot, or another computer also connected to the hotspot

6. Run or edit the code as desired 

## 1.2 Use the existing Raspberry Pi setup by the 2021-2022 team
1. Prerequisites
   1. Another computer running Ubuntu that can generate a wifi hotspot
   2. A computer with [VNC Viewer](https://www.realvnc.com/en/connect/download/viewer/) installed

2. Setup a wifi hotspot on the Ubuntu machine running the high level code
    1. Network SSID: `charlie-XPS-15-9570`
    2. Network Password `charlie-XPS-15-9570`
    3. [Ubuntu Hotspot Help](https://www.fosslinux.com/2576/how-to-create-and-configure-wi-fi-hotspot-in-ubuntu-17-10.htm)

3. Power on the Raspberry Pi (note: the hotspot must be running prior to powering on the pi because of how the VNC server on the Pi Works)
    1. Find the IP Address of the raspberry pi on the hotspot using the command `arp -a` in a terminal on the computer generating the hotspot

4. Launch VNC on either the ubuntu machine, or another computer connected to the hotspot and connect to the ip found in the previous step
    1. Username: Pi
    2. Password: mmmqp2021
    3. [VNC Viewer](https://www.realvnc.com/en/connect/download/viewer/)

6. Run or edit the code as desired



# 2 Code
## 2.1 Runmodes
Edit the runmodes variables in the file mmmqp_eoat/src/EOAT_c.cpp to run the program how you desire

- autosetup: 
  - Determines if the EOAT requires user confirmation prior to initializing. 
  - If you are rewiring or running this for the first time, you should probably set this to false until you check things are wired properly.

- testTooling
  - Determines if the EOAT will perform the preprogrammed motion tests


## 2.2 Manipulator Positioning before starting the program
- Ideal starting position for the manipulators is ~10mm from the outermost positions, although any position **at least 10mm from actuating the inner limit switches** is fine. Starting the program with manipulators less than 10mm from the center may result in lead screw or stepper motor damage. The program should be started WITHOUT TOOLS ATTACHED.
- Instructions assume you cloned as specified above. If you haven't, ignore the first line and navigate to the /mmmqp_eoat/src/runscripts/ folder


## 2.3 Running the program with a local roscore
ie using the EOAT without the ABB 1600. Commands must be issued from the command line on the raspberry pi. See [Section 3.2](https://github.com/cvkittler/Metamorphic-Manufacturing/tree/EOAT_ros#32-commands-being-issued-to-eoat)

Run the following commands in a terminal:
```
cd ~/catkin_ws/src/mmmqp_eoat/src/runscripts
sudo bash run-local.sh
```
to end the program, hit ctrl+c in each of the terminals that pop up


## 2.4 Running the rogram with a remote roscore 
ie using with the ABB1600 under unified ROS control from another machine. Commands can be issued either from the command line on the Raspberry Pi OR another ROS Node
- Connect the Raspberry Pi a wifi hotspot running on the machine running roscore (likely the ubuntu machine used in [Section 1](https://github.com/cvkittler/Metamorphic-Manufacturing/tree/EOAT_ros#1-initial-setup))
- Remote desktop to the Raspberry Pi using the method of your choice

Edit the roscore-program-remote.sh file if necessary (right click -> text editor)
```
#!/bin.bash

source /home/pi/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://[ip of computer running roscore on IRC5 netowrk]
export ROS_IP=[raspi ip on hotpsot]
rosrun mmmqp_eoat mmmqp_eoat_node
read line
```

Run the following commands in a terminal:
```
cd ~/catkin_ws/src/mmmqp_eoat/src/runscripts
sudo bash run-remote.sh
```
to end the program, hit ctrl+c in each of the terminals that pop up

See [README for the ABB_ros](https://github.com/cvkittler/Metamorphic-Manufacturing/tree/ABB-ROS#launch-commands) branch for instructions setting up the remote roscore



# 3 ROS STUFF
## 3.1 Status updates published from EOAT
Topic: "mmm_eoat_position"
Message: Point32

float  |  use
----  |  -----
x  |  left manipulator position (mm)
y  |  right manipulator position (mm)
z  |  status/error code

Status/Error Code  | Meaning
------------- | -------------
0  | Waiting For Instruction
1  | Instruction Execution in Progress
2  | A Manipulator has exceeded its limits
3  | EOAT In Stopped State
4  | EOAT Initializing
5  | Test Routine in Progress
6  | Tool offsets Set. Max Position now set to x y
7  | Invalid command recieved
8  | Calibration In Progress
9  | Unknown Error
911  | Emergency Stop

View this topic by opening a terminal and using the command "rostopic echo mmm_eoat_position"


## 3.2 Commands being issued to EOAT
Topic: "mmm_eoat_command"
Message: Point32

float  |  valid range  |  use
------------- | -------------| -------------
x  |  [0,max*)  | left manipulator position** (mm)
y  |  [0,max*)  | right manipulator position** (mm)
z  |  [1.0,15.0]u[-732]u[911]  | speed/command

*max is automatically calculated by the calibration process. Approximately 133mm from fully open

z command  | Meaning
------------- | -------------
-732  | Calibrate the Manipulator
-111  | set tool offsets to x y
-911  | Execute an Emergency Stop
[valid float]  | Speed for manipulator to follow

*note, if any special codes are present in the z, the x and y entries do not matter provided they are valid for the data type*

Publish to this topic with the command "rostopic pub -r 10 /mmm_eoat_command geometry_msgs/Point32 '{x:[ ], y:[ ], z:[ ]}'"
```
Examples:
//Move to Left = 45mm, Right = 30mm at a speed of 3mm/s
rostopic pub -r 10 /mmm_eoat_command geometry_msgs/Point32 '{x: 45, y: 30, z: 3}'

//Recalibrate the Manipulator
rostopic pub -r 10 /mmm_eoat_command geometry_msgs/Point32 '{x: 45, y: 30, z: -732}'


//Initiate an Emergency Stop
rostopic pub -r 10 /mmm_eoat_command geometry_msgs/Point32 '{x: 45, y: 30, z: 911}'
```



# 4 Raspberry Pi Stuff
## 4.1 Control Signal Wiring
To change these, edit the file  ~/catkin_ws/src/mmmqp_eoat/src/manipulator.h
Use  | BCM Pin #
------------- | -------------
EN_L  | 5
DIR_L  | 6
STEP_L  | 13
EN_R  | 14
DIR_R  | 15
STEP_R  | 18
Left Inner Limit Switch  | 17
Left Outer Limit Switch  | 4
Right Inner Limit Switch  | 22
Right Outer Limit Switch  | 27

![Raspberry Pi 4 Pinout](https://www.etechnophiles.com/wp-content/uploads/2021/01/R-Pi-4-GPIO-Pinout.jpg)

Limit switch wiring:
Terminal  | Connected to
------------- | -------------
Common  | Data pin
Normally Closed  | Ground
Normally Open  | +3.3v


## 4.2 Libraries Used
- [Pigpio](https://abyz.me.uk/rpi/pigpio/examples.html)
- [Signal](https://en.cppreference.com/w/cpp/utility/program/signal)
- iostream
- sstream
- Misc common stuff (time, stdio, stdlib, stdint etc...)



# 5 Random Stuff
## 5.1 Geany Build commands
```
  compile: g++ -Wall -c "%f"

  build: g++ -Wall -o "%e" "%f" -lpigpio

  execute: sudo "./%e"
 ```


# 5.2 Notes
- Errors that occur during the initial calibration are fatal and non recoverable (usually in the form of a locked out manipulator unable to move due to an anomalous sensor reading getting stuck in a forever loop of trying to move while locked out)
- The left outer limit switch intermittenly experiences a glitch where it reports many toggles between closed an open circuits in quick succession if the normally closed terminal is not properly grounded
 
 
 
 # 6 TODO
- [x] Launch Files etc...
- [ ] setup automatic launch on raspi boot
- [x] setup remote roscore procedure and stuffs
- [x] Figure out the automatic configuration of PosLMax and PosRMax
- [x] Fix info being published so it actually publishes while moving (if possible)
- [x] tool offsets
- [x] fix position publishing to give dist from center, not dist from outside
- [ ] Figure out automatic program termination for errors during initial calibration
- [ ] Fix speed control


