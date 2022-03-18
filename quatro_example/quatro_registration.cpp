#include <iostream>
#include <string>
#include <ros/ros.h>
#include <chrono>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "fpfh_manager.hpp"
#include "quatro.hpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr est_cloud(new pcl::PointCloud<pcl::PointXYZ>);

std::string src_path = "/home/beom/coding test/000540.bin";
std::string tgt_path = "/home/beom/coding test/001319.bin";

sensor_msgs::PointCloud2 srcMsg;
sensor_msgs::PointCloud2 tgtMsg;
sensor_msgs::PointCloud2 estMsg;

    // Feature extraction parameters
double voxel_size, normal_radius, fpfh_radius;

    // Quatro parameters
bool   estimating_scale;
int    num_max_iter;
double noise_bound, noise_bound_coeff, gnc_factor, rot_cost_diff_thr;

    //  VISUALIZE
sensor_msgs::PointCloud2 cloud2cloudmsg(pcl::PointCloud<pcl::PointXYZ> cloud_src){
    sensor_msgs::PointCloud2 cloudmsg;
    pcl::toROSMsg(cloud_src, cloudmsg);
    cloudmsg.header.frame_id = "map";
    return cloudmsg;
}

    //  LOAD PCD BIN FILE
pcl::PointCloud<pcl::PointXYZ>::ConstPtr load_cloud(const std::string filename) {
    FILE* file = fopen(filename.c_str(), "rb");
    if (!file) {
        std::cerr << "error: failed to load " << filename << std::endl;
        return nullptr;
    }
    std::vector<float> buffer(1000000);
    size_t num_points = fread(reinterpret_cast<char*>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
    fclose(file);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->resize(num_points);
    for (int i = 0; i < num_points; i++) {
        auto& pt = cloud->at(i);
        pt.x = buffer[i * 4];
        pt.y = buffer[i * 4 + 1];
        pt.z = buffer[i * 4 + 2];
        // pt.intensity = buffer[i * 4 + 3];
    }
    return cloud;
}

    //  VOXELIZATION
void voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_src, pcl::PointCloud<pcl::PointXYZ>& pc_dst, double var_voxel_size){

    static pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(pc_src);
    voxel_filter.setLeafSize(var_voxel_size, var_voxel_size, var_voxel_size);
    voxel_filter.filter(pc_dst);
}

void setParams(
        double noise_bound_of_each_measurement, double square_of_the_ratio_btw_noise_and_noise_bound,
        double estimating_scale, int num_max_iter, double control_parameter_for_gnc,
        double rot_cost_thr, const string& reg_type_name, Quatro<PointType, PointType>::Params &params) {

        params.noise_bound                   = noise_bound_of_each_measurement;
        params.cbar2                         = square_of_the_ratio_btw_noise_and_noise_bound;
        params.estimate_scaling              = estimating_scale;
        params.rotation_max_iterations       = num_max_iter;
        params.rotation_gnc_factor           = control_parameter_for_gnc;
        params.rotation_estimation_algorithm = Quatro<PointType, PointType>::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
        params.rotation_cost_threshold       = rot_cost_thr;

        params.reg_name                  = reg_type_name;
        if (reg_type_name == "Quatro") {
            params.inlier_selection_mode = Quatro<PointType, PointType>::INLIER_SELECTION_MODE::PMC_HEU;
        } 
        else { 
            params.inlier_selection_mode = Quatro<PointType, PointType>::INLIER_SELECTION_MODE::PMC_EXACT; 
        }
    }

Eigen::Matrix4d performQuatro(){
    Quatro<PointType, PointType>         quatro;
    Quatro<PointType, PointType>::Params params;

    setParams(noise_bound, noise_bound_coeff,
            estimating_scale, num_max_iter, gnc_factor, rot_cost_diff_thr, "Quatro", params);
    quatro.reset(params);

    FPFHManager fpfhmanager(normal_radius, fpfh_radius);
    fpfhmanager.flushAllFeatures();
    fpfhmanager.setFeaturePair(src_cloud, tgt_cloud);

    pcl::PointCloud<PointType>::Ptr srcMatched(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr tgtMatched(new pcl::PointCloud<PointType>);
    *srcMatched = fpfhmanager.getSrcKps();
    *tgtMatched = fpfhmanager.getTgtKps();

    quatro.setInputSource(srcMatched);
    quatro.setInputTarget(tgtMatched);
    Eigen::Matrix4d quatro_output;
    quatro.computeTransformation(quatro_output);
    std::cout << quatro_output << std::endl;
    
    return quatro_output;
}

int main(int argc, char** argv){
    ros::init (argc, argv, "teaser_registration");
    ros::NodeHandle nh;

        // Feature extraction parameters
    nh.param<double>("/voxel_size", voxel_size, 0.2);
    nh.param<double>("/FPFH/normal_radius", normal_radius, 0.5);
    nh.param<double>("/FPFH/fpfh_radius", fpfh_radius, 0.75);

        // Quatro parameters
    nh.param<bool>("/Quatro/estimating_scale", estimating_scale, false);
    nh.param<double>("/Quatro/noise_bound", noise_bound, 0.25);
    nh.param<double>("/Quatro/noise_bound_coeff", noise_bound_coeff, 0.99);
    nh.param<double>("/Quatro/rotation/gnc_factor", gnc_factor, 1.39);
    nh.param<double>("/Quatro/rotation/rot_cost_diff_thr", rot_cost_diff_thr, 0.0001);
    nh.param<int>("/Quatro/rotation/num_max_iter", num_max_iter, 50);

    ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("src_cloud", 1);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("tgt_cloud", 1);
    ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2> ("est_cloud", 1);

    *src_cloud = *load_cloud(src_path);
    *tgt_cloud = *load_cloud(tgt_path);

        //  voxelize
    float voxelsize = 0.3;
    pcl::PointCloud<pcl::PointXYZ> src_voxel;
    pcl::PointCloud<pcl::PointXYZ> tgt_voxel;

    voxelize(src_cloud, src_voxel, voxelsize);
    voxelize(tgt_cloud, tgt_voxel, voxelsize);

    src_cloud->clear();
    tgt_cloud->clear();
    *src_cloud = src_voxel;
    *tgt_cloud = tgt_voxel;
        
    Eigen::Matrix4d quatro_output;
    quatro_output = performQuatro();

    pcl::transformPointCloud(*src_cloud, *est_cloud, quatro_output);

    srcMsg = cloud2cloudmsg(*src_cloud);
    tgtMsg = cloud2cloudmsg(*tgt_cloud);
    estMsg = cloud2cloudmsg(*est_cloud);
    ros::Rate loop_rate(0.1);

    while (nh.ok())
    {
        pub1.publish(srcMsg);
        pub2.publish(tgtMsg);
        pub3.publish(estMsg);
        ros::spinOnce ();
        loop_rate.sleep ();
    }

}