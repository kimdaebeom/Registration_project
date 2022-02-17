#include <iostream>
#include <string>
#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/registration/gicp.h>

#include <sensor_msgs/PointCloud2.h>

using namespace std;

sensor_msgs::PointCloud2 cloud2cloudmsg(pcl::PointCloud<pcl::PointXYZ> cloud_src)
  {
    sensor_msgs::PointCloud2 cloudmsg;
    pcl::toROSMsg(cloud_src, cloudmsg);
    cloudmsg.header.frame_id = "map";
    return cloudmsg;
  }

void voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_src, pcl::PointCloud<pcl::PointXYZ>& pc_dst, double var_voxel_size){

    static pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(pc_src);
    voxel_filter.setLeafSize(var_voxel_size, var_voxel_size, var_voxel_size);
    voxel_filter.filter(pc_dst);
}

int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);

  ros::init (argc, argv, "registration");
  ros::NodeHandle nh;

  ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("points1", 1);
  //ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);
  ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2> ("points3", 1);
  ros::Publisher alignpub = nh.advertise<sensor_msgs::PointCloud2> ("align_points", 1);



      // READ PCD FILE

  pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/beom/ouster_ws/src/registration/pcd/kaist_W_entrance.pcd", *cloud1);
  //pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/beom/ouster_ws/src/registration/pcd/kaist_W_indoor_outdoor.pcd", *cloud2);
  pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/beom/ouster_ws/src/registration/pcd/kaist_W_parkinglot.pcd", *cloud3);



      // Rotate 180 degree by initial guess

  float theta = M_PI;
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << 0.0, 0.0, 0.0;
  transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

  pcl::transformPointCloud(*cloud1, *cloud1, transform);



      // VOXELIZATION

  float voxelsize = 0.2;
  pcl::PointCloud<pcl::PointXYZ> tmp1;
  pcl::PointCloud<pcl::PointXYZ> tmp3;

  voxelize(cloud1, tmp1, voxelsize);
  voxelize(cloud3, tmp3, voxelsize);

  *cloud1 = tmp1;
  *cloud3 = tmp3;



      // KD - TREE
      //  ...
      



      //  G-ICP

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setMaxCorrespondenceDistance(0.5);
  gicp.setTransformationEpsilon(0.0003);
  gicp.setMaximumIterations(10000);

  pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);
  gicp.setInputSource(cloud1);
  gicp.setInputTarget(cloud3);
  gicp.align(*align);

  // Set outputs
  Eigen::Matrix4f src2tgt   = gicp.getFinalTransformation();
  double score     = gicp.getFitnessScore();
  bool is_converged = gicp.hasConverged();
  





  sensor_msgs::PointCloud2 msg1;
  //sensor_msgs::PointCloud2 msg2;
  sensor_msgs::PointCloud2 msg3;
  sensor_msgs::PointCloud2 align_msg;

  msg1 = cloud2cloudmsg(*cloud1);
  //msg2 = cloud2cloudmsg(*cloud2);
  msg3 = cloud2cloudmsg(*cloud3);
  align_msg = cloud2cloudmsg(*align);

  ros::Rate loop_rate(0.1);

  while (nh.ok())
  {
    //msg->header.stamp = ros::Time::now().toNSec();
    pub1.publish(msg1);
    //pub2.publish(msg2);
    pub3.publish(msg3);
    alignpub.publish(align_msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
