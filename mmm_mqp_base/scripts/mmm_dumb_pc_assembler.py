#!/usr/bin/env python
from copy import deepcopy
from email.header import Header
import rospy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud,ChannelFloat32
from std_srvs.srv import Empty

pub = None
savedPoints = []
combine = False
def inputCallback(data):
    global combine
    if combine:
        print("mmm_dumb_pc_assembler: I FOUND A POINTCLOUDS")
        global savedPoints
        for p in data.points:
            point = []
            point=(p.x,p.y,p.z,)
            savedPoints.append(point)

def startCallback(emptyData):
    global combine
    print("mmm_dumb_pc_assembler: I AM LISTENING FOR POINTCLOUDS")
    combine = True
    return []
    
def assembleCallback(emptyData):
    global combine, savedPoints, pub
    combine = False
    localPoints = deepcopy(savedPoints)
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "mmm_pointcloud_frame"
    #make point cloud
    currPointCloud = PointCloud()
    currPointCloud.header = header
    for p in localPoints:
        currPointCloud.points.append(Point32(p[0],p[1],p[2]) )
        currPointCloud.channels.append(ChannelFloat32())
    print("Publishing Combined Point Cloud" + str(len(currPointCloud.points)))
    pub.publish(currPointCloud)
    return []

def mainLoop():
    global pub
    pub = rospy.Publisher("/mmm_pointcloud_combined", PointCloud, queue_size=1)
    while not rospy.is_shutdown():
        rospy.Subscriber("/mmm_pointcloud", PointCloud, inputCallback)
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('mmm_dumb_pc_assembler')
    s = rospy.Service('mmm_start_pc_listen', Empty, startCallback)
    s = rospy.Service('mmm_assemble_pc', Empty, assembleCallback)
    mainLoop()