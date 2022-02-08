#!/usr/bin/env python
from copy import deepcopy
from email.header import Header
import rospy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud2,PointCloud,ChannelFloat32
import ros_numpy as np
from numpy import zeros
from std_srvs.srv import Empty

pub = rospy.Publisher("/mmm_pointcloud", PointCloud2, queue_size=1)
savedPoints = None
def callback(data):
    global savedPoints
    pc = np.numpify(data)
    points = zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    savedPoints = data

def sendPointCloud(msg):
    global savedPoints
    localPoints = deepcopy(savedPoints)
    #make header
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "mmm_pointcloud_frame"
    #make point cloud
    currPointCloud = PointCloud()
    currPointCloud.header = header
    # for i in range(0,len(localPoints[:,0]), 50):
    #     currPointCloud.points.append(Point32(localPoints[i,0],localPoints[i,1],localPoints[i,2]))
    #     currPointCloud.channels.append(ChannelFloat32())
    # print("Publishing Point Cloud" + str(len(currPointCloud.points)))
    pub.publish(localPoints)
    return []

def mainLoop():
    while not rospy.is_shutdown():
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback)
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('mmm_pc2_to_pc')
    s = rospy.Service('mmm_processed_pointCloud', Empty, sendPointCloud)
    mainLoop()