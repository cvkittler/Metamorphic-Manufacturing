#!/usr/bin/env python
from copy import deepcopy
from email.header import Header
from itertools import count
import rospy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud2,PointCloud,ChannelFloat32
import sensor_msgs.point_cloud2 as pc2
import ros_numpy as np
from numpy import zeros
from std_srvs.srv import Empty
from tf import TransformListener

pub = None
mostRecentCloud = None
listener = None

def callback(data):
    global mostRecentCloud
    pc = np.numpify(data)
    points = zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    mostRecentCloud = data

def sendPointCloud(msg):
    global mostRecentCloud, pub, listener
    localPoints = deepcopy(mostRecentCloud)
    #make header
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "mmm_pointcloud_frame"
    #make point cloud
    currPointCloud = PointCloud()
    currPointCloud.header = header
    counter = 0
    for p in pc2.read_points(localPoints, field_names = ("x", "y", "z"), skip_nans=True):
        counter += 1
        if(counter % 100 == 0):
            currPointCloud.points.append(Point32(p[0],p[1],p[2]) )
            currPointCloud.channels.append(ChannelFloat32())
    print("Publishing Point Cloud with " + str(len(currPointCloud.points))+ "Points")
    dataLocal = listener.transformPointCloud("/base_link", currPointCloud)
    pub.publish(dataLocal)
    return []

def mainLoop():
    global pub, listener
    listener = TransformListener()
    pub = rospy.Publisher("/mmm_pointcloud", PointCloud, queue_size=1)

    while not rospy.is_shutdown():
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback)
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('mmm_pc2_to_pc')
    s = rospy.Service('mmm_processed_pointCloud', Empty, sendPointCloud)
    mainLoop()