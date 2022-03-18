#include <iostream>
#include <string>
#include <ros/ros.h>
#include <chrono>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <teaser/ply_io.h>
#include <teaser/registration.h>
#include <teaser_utils/feature_matcher.h>
#include <conversion.hpp>

#define NOISE_BOUND 0.3

pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr est_cloud(new pcl::PointCloud<pcl::PointXYZ>);

std::string src_path = "/home/beom/coding test/000540.bin";
std::string tgt_path = "/home/beom/coding test/001319.bin";

sensor_msgs::PointCloud2 srcMsg;
sensor_msgs::PointCloud2 tgtMsg;
sensor_msgs::PointCloud2 estMsg;

//boost::shared_ptr<PatchWork<PointType> > patchwork;

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

int main(int argc, char** argv){
    ros::init (argc, argv, "teaser_registration");
    ros::NodeHandle nh;

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
        
    //Compute FPFH
    teaser::FPFHEstimation fpfh;
    teaser::PointCloud src_teaser;
    teaser::PointCloud tgt_teaser;
    pcl2teaser(*src_cloud, src_teaser);
    pcl2teaser(*tgt_cloud, tgt_teaser);

    auto src_fpfh = fpfh.computeFPFHFeatures(src_teaser, 0.5, 0.75);
    auto tgt_fpfh = fpfh.computeFPFHFeatures(tgt_teaser, 0.5, 0.75);

    teaser::Matcher matcher;
    auto correspondences = matcher.calculateCorrespondences(
        src_teaser, tgt_teaser, *src_fpfh, *tgt_fpfh, false, true, false, 0.0);

    // TEASER++ registration
    // Solver parameters
    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound = NOISE_BOUND;
    params.cbar2 = 1;
    params.estimate_scaling = false;
    params.rotation_max_iterations = 500;
    params.rotation_gnc_factor = 1.4;
    params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_cost_threshold = 1e-5;

    // Solve with TEASER++
    teaser::RobustRegistrationSolver solver(params);
    solver.solve(src_teaser, tgt_teaser, correspondences);

    auto solution = solver.getSolution();
    Eigen::Matrix4d teaserMat;
    teaserMat = Eigen::Matrix4d::Identity();
    teaserMat.block<3,3>(0,0) = solution.rotation;
    teaserMat.block<3,1>(0,3) = solution.translation;
    std::cout << solution.rotation << std::endl;
    std::cout << solution.translation << std::endl;

    pcl::transformPointCloud(*src_cloud, *est_cloud, teaserMat);

       
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