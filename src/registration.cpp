#include <iostream>
#include <string>
#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>

#include <sensor_msgs/PointCloud2.h>

using namespace std;

sensor_msgs::PointCloud2 cloud2cloudmsg(pcl::PointCloud<pcl::PointXYZ> cloud_src)
  {
    sensor_msgs::PointCloud2 cloudmsg;
    pcl::toROSMsg(cloud_src, cloudmsg);
    cloudmsg.header.frame_id = "map";
    return cloudmsg;
  }

int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  ros::init (argc, argv, "registration");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("points", 1);

  pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/beom/ouster_ws/src/registration/pcd/kaist_W_indoor_outdoor.pcd", *cloud);
  sensor_msgs::PointCloud2 msg;

  msg = cloud2cloudmsg(*cloud);

  ros::Rate loop_rate(4);


  while (nh.ok())
  {
    //msg->header.stamp = ros::Time::now().toNSec();
    pub.publish(msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
