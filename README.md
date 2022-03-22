# Metamorphic-Manufacturing-ABB

This branch is ment to be the base/main computer, it is where `rocore` is expected to be run from. It handles controllng the othersub systems

## Connected Systems / Network layout
System  | Use | IPv4 Adress 
------------- | ------------- | -------------
Main Computer | Control other systems | 192.168.100.104
ABB Robot Arm | The Robotic arm system that is the main motion system | 192.168.100.100
RaspberryPi | The rPi runs the node that controlls the end of arm tooling | 10.42.0.123 (may change)

### Launch commands
#### Abb arm hooked up with the specifications below
  ```export ROS_MASTER_URI=http://192.168.100.104:11311```\
  ```export ROS_IP=192.168.100.104```\
  ```roslaunch mmm_mqp_base main.launch sim:=false```
#### Abb not connected
```roslaunch mmm_mqp_base main.launch sim:=true```

### Command Language
The open loop command files (.mmm) can be used to have the robot exacute a set of preprogrammed commands\
I suggest storing these files in the mmm_mqp_base/commandFiles dir, there should be some premade files
#### Syntax:
Each command must start with one of the commands and have to additinal white space\
Each command and all of its fields should be sperated by a : without white space\
Each command line must end with a ; \
optionally the next command can be on the same line or next line (recommend for readablity in the GUI)\
optionally can add a word or phrase after the last ; (ie. END) which will be printed as the last command in the terminal
#### Commands
mmm file commad  | Fields | Units | Explanation
------------- | ------------- | ------------- | -------------
POSE | X:Y:Z:rX:rY:rZ | Meters | Move the end of the robot to the pose (X,Y,Z) with rotation (rX,rY,rZ)
JOINT | j1:j2:j3:j4:j5:j6 | Degrees | Moves the robot so that each joint is at a the angle specified (Much faster than POSE)
EOAT | Left:Right:Speed | Millimeters | Moves the EOAT fingers to distance (Left, Right) from center for each finger at speed (Speed). Speed of -911 is for Estop. Speed of -732 is for Calibrate. Speed of -111 for setting offsets (might make diff command in the future)
STEP | Distance:Direction | Meters | Moves the (X,Y,Z) Pose of the robot by Distance in the Direction. Direction can be (X, -X, Y, -Y, Z, -Z). Negitive distances are also supported.
WAIT | Time | Seconds | Have the code wait for (Time) seconds
HOME | None | None | Homes the robot by calling JOINT:0:0:0:0:0:0
SCAN | None | None | Calls the scan service which starts a scan routine
COMMENT | None | None | Allows you to leave a comment in the file to help with read ablity 
