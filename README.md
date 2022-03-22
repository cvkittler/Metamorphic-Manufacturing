# Metamorphic-Manufacturing-ABB

## Launch commands
### Abb arm hooked up with the specifications below
```roslaunch mmm_mqp_base main.launch sim:=false```
### Abb not connected
```roslaunch mmm_mqp_base main.launch sim:=true```
mmm file commad  | Fields | Units | Explanation
------------- | ------------- | ------------- | -------------
POSE | X:Y:Z:rX:rY:rZ | Meters | Move the end of the robot to the pose (X,Y,Z) with rotation (rX,rY,rZ)
JOINT | j1:j2:j3:j4:j5:j6 | Degrees | Moves the robot so that each joint is at a the angle specified (Much faster than POSE)
EOAT | Left:Right:Speed | Millimeters | Moves the EOAT fingers to distance (Left, Right) from center for each finger at speed (Speed). Speed of -911 is for Estop. Speed of -732 is for Calibrate. Speed of -111 for setting offsets (might make diff command in the future)
STEP | Distance:Direction | Meters | Moves the (X,Y,Z) Pose of the robot by Distance in the Direction. Direction can be (X, -X, Y, -Y, Z, -Z). Negitive distances are also supported.
WAIT | Time | Seconds | Have the code wait for (Time) seconds
HOME | None | None | Homes the robot by calling JOINT:0:0:0:0:0:0
SCAN | None | None | Calls the scan service which starts a scan routine
