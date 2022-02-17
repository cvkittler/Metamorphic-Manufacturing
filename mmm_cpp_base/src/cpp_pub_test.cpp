#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/point_cloud.h"

ros::Publisher pub;

sensor_msgs::PointCloud2 loadPointCloud(ros::Publisher pub)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // you have an object already, eg with pcl::PointXYZRGB points
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    pcl::io::loadPCDFile ("pointClouds/test_pcd.pcd", cloud);

    // and just publish the object directly
    pub.publish(cloud);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "cpp_pub_test");
    ros::NodeHandle nh;

    // create a templated publisher
    ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("test_publisher", 1);
    //load then publish a point cloud
    loadPointCloud(pub);

    // Spin
    ros::spin ();
}