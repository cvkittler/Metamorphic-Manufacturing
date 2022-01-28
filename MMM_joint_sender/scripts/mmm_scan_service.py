#!/usr/bin/env python

from copy import deepcopy
from time import sleep
import rospy
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
import moveit_commander
import sys
from math import pi
from tf.transformations import quaternion_from_euler

# initilize global variables
group = None
group_name = "manipulator"
# points to scan
points = [[0.,      -0.85,  0.85,   90.,    70,     0.],    #top
          [0.373,   -0.85,  0.747,  90,     90.,    0.],    #right 1
          [0.59,    -0.85,  0.59,   90,     110.,   0.],    #right 2
          [0.373,   -0.85,  0.747,  90,     90.,    0.],   #right 1
          [0.,      -0.85,  0.85,   90.,    70,     0.], #top
          [-0.373,  -0.85,  0.747,  90,     50.,    0.],  #left 1
          [-0.59,   -0.85,  0.59,   90,     30.,    0.],      #left 2
          [-0.373,  -0.85,  0.747,  90,     50.,    0.]]  #left 1

# service request handler
def handle_scan(req):
    print("Request Recived")
    global points
    # j is froward backwards
    for j in range(-1,1,1):
        for i in range(len(points)):
            target_pose = deepcopy( points[i])
            target_pose[1] += (j * 0.15)
            moveToPoint(target_pose)
            print("Reached Pose " + str(i + 1))
            sleep(1)
    moveToPoint([0., -0.85, 0.85, -180., 0, -110.])
    return []

def moveToPoint(pose):
    # create variables
    pose_goal = Pose()
    pose_start = group.get_current_pose()
    # convert euler angeles to quaternion
    q = quaternion_from_euler(float(pose[3]) * (pi/180),float(pose[4]) * (pi/180),float(pose[5]) * (pi/180))
    # create the goal pose
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]
    pose_goal.position.x = float(pose[0])
    pose_goal.position.y = float(pose[1])
    pose_goal.position.z = float(pose[2])
    # set the goal pose
    group.set_pose_target(pose_goal)
    # compute the cartesian path (strait)
    (plan, fraction) = group.compute_cartesian_path([pose_start.pose,pose_goal],0.01,0.0)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
def scan_routine():
    rospy.init_node('mmm_scan_service')
    s = rospy.Service('mmm_scan_service', Empty, handle_scan)
    print("Ready to Scan")
    rospy.spin()

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander(group_name)
    scan_routine()