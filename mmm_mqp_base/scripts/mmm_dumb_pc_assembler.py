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
            # the numbers that are +/- are from the final pose of the scaning service 
            point=((p.x - .2), (p.y + .85), (p.z -.85))
            if checkIfPointValid(point): # some real basic cleaning
                savedPoints.append(point)

# filter out all points outside the work area
def checkIfPointValid(point):
    zCutoff = 0.02
    xMin = -0.3
    xMax = 0.5
    yMin = -1.25
    yMax = -0.75
    #check Z
    if(point[2] > zCutoff):
        return False
    # check y
    if (yMax > point[1] < yMin):
        return False
    # check x
    if (xMax > point[0] < xMin):
        return False
    return True

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