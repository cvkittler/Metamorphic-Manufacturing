#!/usr/bin/env python

from time import sleep
import rospy
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
import moveit_commander
import sys
from math import pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler

group = None
group_name = "manipulator"
points = [[0., -0.85, 0.85, -180., 0, -110.],
          [0.373, -0.85, 0.747, -26.5, 180., 70.],
          [0.59, -0.85, 0.59, -45, 180., 70.],
          [0.747, -0.85, 0.373, -63.5, 180., 70.],
          [0.59, -0.85, 0.59, -45, 180., 70.],
          [0.373, -0.85, 0.747, -26.5, 180., 70.],
          [0., -0.85, 0.85, -180., 0, -110.],
          [-0.373, -0.85, 0.747, 26.5, 180., 70.],
          [-0.59, -0.85, 0.59, 45, 180., 70.],
          [-0.747, -0.85, 0.373, 63.5, 180., 70.]]

def handle_scan(req):
    print("Request Recived")
    print("Joint Angles Submitted")
    global points
    group.clear_pose_targets()
    joint_goal = group.get_current_joint_values()
    for i in range(len(points)):
        pose_goal = Pose()

        pose_start = group.get_current_pose()
        
        q = quaternion_from_euler(float(points[i][3]) * (pi/180),float(points[i][4]) * (pi/180),float(points[i][5]) * (pi/180))
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]
        pose_goal.position.x = float(points[i][0])
        pose_goal.position.y = float(points[i][1])
        pose_goal.position.z = float(points[i][2])

        group.set_pose_target(pose_goal)
        (plan, fraction) = group.compute_cartesian_path([pose_start.pose,pose_goal],0.01,0.0)
        group.go(wait=True)
        print("Reached Pose " + str(i + 1))
        sleep(1)
        group.stop()
    return []

def scan_routine():
    rospy.init_node('mmm_scan_service')
    s = rospy.Service('mmm_scan_service', Empty, handle_scan)
    print("Ready to Scan")
    rospy.spin()

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander(group_name)
    scan_routine()