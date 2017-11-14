#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <pcl/filters/conditional_removal.h>

std_msgs::Float32MultiArray msgOut;
ros::Publisher *pubPtr;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (temp_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.1, 0.0);
    pass.filter(*filtered);

    pcl::PassThrough<pcl::PointXYZRGB> pass1;
    pass1.setInputCloud (filtered);
    pass1.setFilterFieldName("z");
    pass1.setFilterLimits(-1.0, 3.0);
    pass1.filter(*filtered1);

    msgOut.data.clear();
    for (int i = 0; i < filtered1 -> width; i++){
        msgOut.data.push_back(filtered1->at(i).z);
        msgOut.data.push_back(filtered1->at(i).x);
        msgOut.data.push_back(filtered1->at(i).r);
        msgOut.data.push_back(filtered1->at(i).g);
        msgOut.data.push_back(filtered1->at(i).b);
    }
    pubPtr->publish(msgOut);
}

int main (int argc, char** argv)
{

  ros::init (argc, argv, "PCL_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points_drop", 1, cloud_cb);
  pubPtr = new ros::Publisher(nh.advertise<std_msgs::Float32MultiArray>("ewa/przekroj", 1000));
  ros::spin ();
}
