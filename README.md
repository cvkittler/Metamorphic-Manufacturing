# Installing and Setup
## OS and ROS
Running ROS Melodic on Raspberry Pi OS. Image here: http://sar-lab.net/teaching/ros-images-for-raspberry-pi/

Enable the GUI

Install Pigpio library (if its not installed, cant remember)

Clone THIS BRANCH such that the package is in ~/catkin_ws/src/ (ie you get ~/catkin_ws/src/mmmqp_eoat/src/EOAT_c.cpp etc...)



## Code
Edit the runmodes variables in the file mmmqp_eoat/src/EOAT_c.cpp to run the program how you desire

- autosetup: 
  - Determines if the EOAT requires user confirmation prior to initializing. 
  - If you are rewiring or running this for the first time, you should probably set this to false until you check things are wired properly.
  - If you are using manual mode (see variable "Jogging") it doesnt matter what this is set as.
  - If you are using automatic mode (see variable "Jogging") this should be set as true.

- testTooling
  - Determines if the EOAT will perform the preprogrammed motion tests

# Running the code
- Ideal starting position for the manipulators is ~10mm from the outermost positions, although any position at least 10mm from innermost position is fine. 
- Instructions assume you cloned as specified above. If you haven't, ignore the first line and navigate to the /mmmqp_eoat/src/runscripts/
- Run the situation specific commands in a terminal
- In the event of an unresolvable error, hit ctrl+c on the terminal windows that pop up

## Local roscore 
ie using the EOAT without the ABB 1600
```
cd ~/catkin_ws/src/mmmqp_eoat/src/runscripts
sudo bash run-local.sh
```

## Remote roscore 
ie using with the ABB1600 under unified ROS control from another machine
```
cd ~/catkin_ws/src/mmmqp_eoat/src/runscripts
sudo bash run-remote.sh
```

# ROS STUFF
## Status updates published from EOAT
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



## Commands being issued to EOAT
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

# Raspberry Pi Stuff
## Wiring Pinout
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

# Random Stuff
## Geany Build commands
```
  compile: g++ -Wall -c "%f"

  build: g++ -Wall -o "%e" "%f" -lpigpio

  execute: sudo "./%e"
 ```
 
 # TODO
- [x] Launch Files etc...
- [ ] setup automatic launch on raspi boot
- [x] setup remote roscore procedure and stuffs
- [x] Figure out the automatic configuration of PosLMax and PosRMax
- [x] Fix info being published so it actually publishes while moving (if possible)
- [x] tool offsets
- [ ] fix position publishing to give dist from center, not dist from outside
