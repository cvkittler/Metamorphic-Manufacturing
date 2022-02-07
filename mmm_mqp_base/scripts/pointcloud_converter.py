#!/usr/bin/env python
import imp
import rospy
from sensor_msgs.msg import PointCloud2,PointCloud
import ros_numpy as np
from numpy import zeros, amax
from std_srvs.srv import Empty

pub = rospy.Publisher("/mmm_pointcloud", PointCloud, queue_size=1)
currPointCloud = PointCloud()

def callback(data):
    pc = np.numpify(data)
    points = zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    # print(points.shape)
    # points = points[points[:,2] > 0,:]
    # print(points.shape)
    # print(amax(points[:,2]))
    # print("x:" + str(len(points[:,0])) + " y:" + str(len(points[:,1])) +" z:" + str(len(points[:,2])))
    currPointCloud = PointCloud()
    currPointCloud.points = points

def sendPointCloud(msg):
    print("Publishing Point Cloud")
    pub.publish(currPointCloud)
    return []

def mainLoop():
    while not rospy.is_shutdown():
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback)
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('mmm_pc2_to_pc')
    s = rospy.Service('mmm_processed_pointCloud', Empty, sendPointCloud)
    mainLoop()