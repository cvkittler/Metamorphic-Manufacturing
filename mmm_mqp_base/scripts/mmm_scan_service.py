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
start_pc_listening = None
assemble_scans = None
publish_cloud_service = None
group_name = "manipulator"

# points to scan
points = [[0.2,      -0.85,  0.85,   90.,    70,     0.],#top
          [0.79,    -0.85,  0.59,   90,     110.,   0.],#right
          [0.2,    -1.3,  0.7,   165,     50.,   75.],#out
          [-0.39,   -0.85,  0.59,   90,     30.,    0.],#left
          [0.0,   -0.65,  0.7,   160,     50.,    -120.]]#in

# service request handler
running = False
def handle_scan(msg):
    global running
    if not running:
        running = True
        global points, publish_cloud_service,assemble_scans,start_pc_listening
        start_pc_listening()
        print("Request Recived")
        for i in range(2):
            target_pose = deepcopy( points[i])
            moveToPoint(target_pose)
            print("Reached Pose " + str(i + 1))
            sleep(2)
            publish_cloud_service()
            sleep(2)
        moveToPoint([0.2, -0.85, 0.85, 90., 250., 0.])
        resp = assemble_scans()
        running = False
    

def moveToPoint(pose):
    group.stop()
    group.clear_pose_targets()
    # create variables
    pose_goal = Pose()
    pose_start = group.get_current_pose()
    # convert euler angeles to quaternion
    q = quaternion_from_euler(float(pose[3]) * (pi/180),float(pose[4]) * (pi/180),float(pose[5]) * (pi/180))
    # create the goal pose
    pose_goal.position.x = float(pose[0])
    pose_goal.position.y = float(pose[1])
    pose_goal.position.z = float(pose[2])
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]
    # set the goal pose
    group.set_pose_target(pose_goal)
    # compute the cartesian path (strait)
    (plan, fraction) = group.compute_cartesian_path([pose_start.pose,pose_goal],0.01,0.0)
    group.go(wait=True)
    
def scan_routine():
    rospy.init_node('mmm_scan_service')
    global pub, assemble_scans, publish_cloud_service, start_pc_listening
    rospy.wait_for_service('mmm_processed_pointCloud')
    rospy.wait_for_service('mmm_start_pc_listen')
    rospy.wait_for_service('mmm_assemble_pc')
    publish_cloud_service = rospy.ServiceProxy("mmm_processed_pointCloud", Empty)
    start_pc_listening =rospy.ServiceProxy("mmm_start_pc_listen", Empty)
    assemble_scans = rospy.ServiceProxy("mmm_assemble_pc", Empty)
    s = rospy.Service('mmm_scan_service', Empty, handle_scan)
    print("Ready to Scan")
    rospy.spin()

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander(group_name)
    scan_routine()