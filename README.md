# Metamorphic-Manufacturing-ABB

This branch is ment to be the base/main computer, it is where `rocore` is expected to be run from. It handles controllng the othersub systems\
This code is devloped for and on Ubuntu 20.04 and ROS Melodic.

### Launch commands
#### ABB arm hooked up with the specifications below
  ```export ROS_MASTER_URI=http://192.168.100.104:11311```\
  ```export ROS_IP=192.168.100.104```\
  ```roslaunch mmm_mqp_base main.launch sim:=false```
#### ABB not connected
```roslaunch mmm_mqp_base main.launch sim:=true```

## Connected Systems / Network layout
System  | Use | IPv4 Adress/Connection Type 
------------- | ------------- | -------------
Main Computer | Control other systems | 192.168.100.104
ABB Robot Arm | The Robotic arm system that is the main motion system | 192.168.100.100
RaspberryPi | The rPi runs the node that controlls the end of arm tooling | 10.42.0.123 (may change)
RealSense Camera | Is how the robot scans and gets point clouds into the system | USB 3.2 to Main Computer

## How to connect everything

RaspberryPi connects to a hotspot being run on the Main Computer
Settings > Wi-Fi > (three bar menu in the top right) > Turn On Wi-Fi Hotspot... > Turn On\
[Hotspot Tutorial](https://www.fosslinux.com/2576/how-to-create-and-configure-wi-fi-hotspot-in-ubuntu-17-10.htm)

SSID | Password
------------- | -------------
charlie-XPS-15-9570 | charlie-XPS-15-9570

ABB Robot Arm connects via ethernet cable to the switch under the table that the arm is mounted too
There is no DHCP server on the LAN network so the IPv4 adress for the Main computer needs to be manully set

Address | Netmask
------------- | -------------
192.160.100.104 | 255.255.255.255

## GUI 
![alt text](https://github.com/cvkittler/Metamorphic-Manufacturing/blob/images-for-readme/Screenshot%20from%202022-03-22%2014-17-15.png)
The GUI looks like this as of (03/22/2022) 
#### Send Work Space / Joint Space Targets
  * This section of the gui can be used to send a target pose (with eular rotation) for the wrist of the ABB robotic arm\
  * Each feild needs to be filled in with a point 
  * Pushing the 'Send Target Position' button will use the values in the feilds to create a target for the arm to move too
  * Pushing the 'Switch To Sending Joint Goals' switch the target goal from a work space goal to a joint space goal
  * NOTE: Joint Space goals are much faster for the trajectory planner
#### Step Pose/Angle
  * The Stepping area is used to move the robot's pose/orientation relitive to its current pose/orientation
  * For Pose Stepping
    * Up/Down is in charge of the X/-X stepping (home pose has a x of about 0.85 m)
    * Left/Right is in charge of the Y/-Y stepping (home pose has a y of 0)
    * Diagnol is in charge of the Z/-Z stepping (Hight)
  * For Angle Stepping
    * Up/Down is in charge of the Pitch stepping
    * Left/Right is in charge of the Yaw stepping
    * Diagnol is in charge of the Role stepping
  * Both have a home button in the middle that homes the robot
#### Current Joint Angles/Workspace Pose
  * Is a readout for both the joint angles and the pose and orientation for the robot
#### End of Arm Tooling
  * The Current finger pose should display the EOAT finger's distances from the center (updates about every second)
  * The Send finger target pose is used to send a target pose for the EOAT fingers, the speed should never be negitive
    *  Send the target location for each finger with the 'Send Target Pose' button
  * The Tool offset area has both a display for the current tool offsets and a feild for setting a new tool offset for the EOAT fingers
    *  Send the new tool offset with the 'Set Tool Offsets Button'
  *  The MISC. EOAT section is where everything else for the EOAT control is
    *  Current state reflects what the EOAT is currently doing
      *  Some of the EOAT buttons will be disabled while the EOAT is not ready to recieve another command
    *  The Calibrate Button sends a command to the EOAT to re-home the fingers
    *  The ESTOP button sends an emergnacy stop signal to the EOAT
#### Automated Functions
  * The Scan button sends a service request that starts the scanning routine which publishs a pointcloud scan when done (if the Realsense camera is plugged into the main computer)
  * Command File is in charge of open loop motion of the robot
    * The Load file button will open a file broser in the `~/catkin_ws/src/mmm_mqp_base/commandFiles` and only show .mmm files
    * Once opened A header and the contencce of the file will be displayed in the text area.
    * The 'Run File' will start running the file with path of the opened file (note if a file is opened and then edited the edited version will be run)
    * The 'Stop File' will stop the exacution of the file once the current command is done
### Command Language
The open loop command files (.mmm) can be used to have the robot exacute a set of preprogrammed commands\
I suggest storing these files in the `mmm_mqp_base/commandFiles` dir, there should be some premade files
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

## Nodes run with main.launch 
/camera/realsense2_camera\
/camera/realsense2_camera_manager\
/industrial_robot_simulator\
/joint_trajectory_action\
/mmm_dumb_pc_assembler\
/mmm_mqp_base_joint_sender\
/mmm_mqp_base_scan_service\
/mmm_pc2_to_pc\
/mmm_pointcloud_connector\
/move_group\
/move_group_commander_wrappers_1647975900320271152\
/move_group_commander_wrappers_1647975900464284549\
/robot_state_publisher\
/rosout\
/rviz_charlie_XPS_15_9570_29076_5973846896081490963\
/world_origin

## Main ROS Packages we made
mmm_mqp_base\
mmm_cpp_base\
abb_irb1600_6_145_support
