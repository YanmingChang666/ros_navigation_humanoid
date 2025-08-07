/**
 * @file point_cloud_display.cpp
 * @author gaohao zhou (gaohaozhou@deepglint.com)
 * @brief 
 *         这个文件用来将本地点云地图文件显示到rviz中
 * 
 * @version 0.1
 * @date 2025-04-14
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_loader");
    ros::NodeHandle nh;
    ros::NodeHandle private_hd("~");

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("map_point_cloud", 1);

    std::string frame_id;
    std::string publish_topic;
    std::string file_path;
    float z_min, z_max, voxel_filte_size, pub_rate;
    // Step1. 从参数服务器中加载参数
    private_hd.param<std::string>("publish_topic", publish_topic, "map_point_cloud");
    private_hd.param<std::string>("frame_id", frame_id, "map");
    private_hd.param<std::string>("file_path", file_path, "");
    private_hd.param<float>("z_min", z_min, 0.0);
    private_hd.param<float>("z_max", z_max, 2.0);
    private_hd.param<float>("voxel_filte_size", voxel_filte_size, 0.05);
    private_hd.param<float>("pub_rate", pub_rate, 0.5);

    // Step1. 加载点云文件
    if ("" == file_path){
        ROS_WARN("File path is not appoint, do not display.");
        return 0;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
        ROS_ERROR("Couldn't read point cloud file [%s]",file_path.c_str());
        return -1;
    }

    // Step2. Z轴范围过滤
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);  // Z轴：0 ~ 2 米

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pass.filter(*filtered_cloud);
    ROS_INFO("After Z filter: %zu points", filtered_cloud->points.size());

    // Step3. 体素滤波降采样
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(filtered_cloud);
    voxel.setLeafSize(voxel_filte_size, voxel_filte_size, voxel_filte_size);  // 体素大小：5cm立方体

    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    voxel.filter(*voxel_filtered_cloud);
    ROS_INFO("After VoxelGrid filter: %zu points", voxel_filtered_cloud->points.size());

    // Step4. 转换为ROS消息
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*voxel_filtered_cloud, output);
    output.header.frame_id = "map";

    ros::Rate loop_rate(pub_rate);
    while (ros::ok()) {
        output.header.stamp = ros::Time::now();
        pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
