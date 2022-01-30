# ROS STUFF
## Launching
###### In Terminal 1:
```
sudo kill all pigpiod
roscore
```
*note dont run roscore if its being run elsewhere* will update once this is figured out

###### In Terminal 2:
```
sudo su
source catkin_ws/devel/setup.bash
rosrun mmmqp_eoat mmmqp_eoat_node
```



## Information being published to the central code
Topic: "mmm_eoat_position"
Message: Point32
```
x is left manipulator position in mm
y is right manipulator position in mm
z is status/error code
```
Status/Error Code  | Meaning
------------- | -------------
0  | Waiting For Instruction
1  | Instruction Execution in Progress
2  | A Manipulator has exceeded its limits
3  | EOAT In Stopped State
4  | EOAT Initializing
5  | Test Routine in Progress
7  | Invalid command recieved
8  | Calibration In Progress
9  | Unknown Error
911  | Emergency Stop

View this topic by opening a terminal and using the command "rostopic echo mmm_eoat_position"



## Commands being issued to EOAT
Topic: "mmm_eoat_command"
Message: Point32
```
x is left manipulator position in mm. Valid range: 0.01 to Maximum Closed value (calculated by code. Approx 130mm)
y is right manipulator position in mm. Valid range: 0.01 to Maximum Closed value (calculated by code. Approx 130mm)
z is speed/Special Statuses. Valid range: 1.0 to 15.0
```
z Status/Error Code  | Meaning
------------- | -------------
-732  | Calibrate the Manipulator
911  | Execute an Emergency Stop
[any other float]  | Speed for manipulator to follow

*note, if any special codes are present in the z, the x and y entries do not matter provided they are valid for the data type*

Publish to this topic with the command "rostopic pub -r 10 /mmm_eoat_command geometry_msgs/Point32 '{x:[ ], y:[ ], z:[ ]}'"
```
Examples:
//Move to Left = 45mm, Right = 30mm at a speed of 3mm/s
rostopic pub -r 10 /mmm_eoat_command geometry_msgs/Point32 '{x:45, y:30, z:3}'

//Recalibrate the Manipulator
rostopic pub -r 10 /mmm_eoat_command geometry_msgs/Point32 '{x:45, y:30, z:-732}'


//Initiate an Emergency Stop
rostopic pub -r 10 /mmm_eoat_command geometry_msgs/Point32 '{x:45, y:30, z:911}'
```


# Random Crap
## Geany Build commands
```
  compile: g++ -Wall -c "%f"

  build: g++ -Wall -o "%e" "%f" -lpigpio

  execute: sudo "./%e"
 ```
