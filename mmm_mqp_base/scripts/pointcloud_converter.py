#!/usr/bin/env python
import rospy
import pcl
from sensor_msgs.msg import PointCloud2,PointCloud
import ros_numpy as np

rospy.init_node('pc2_to_pc')

pub = rospy.Publisher("/mmm_pointcloud", PointCloud, queue_size=1)

def callback(data):
    print("Converting")
    pc = np.numpify(data)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    p = pcl.PointCloud(np.array(points, dtype=np.float32))
    pub.publish(p)


while not rospy.is_shutdown():
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback)
    rospy.spin()