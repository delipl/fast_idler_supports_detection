#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <objects_detection/cluster_extraction.hpp>
#include <objects_detection/down_sampling.hpp>
#include <objects_detection/ground_removal.hpp>
#include <objects_detection/outlier_removal.hpp>
#include <objects_detection/utils.hpp>

using namespace std::chrono_literals;

class ObjectDetection : public rclcpp::Node {
   public:
    ObjectDetection();

   private:
    void declare_parameters();
    void get_parameters();

    std::string pointcloud_topic_name;

    double roll;
    double pitch;
    double yaw;

    double x;
    double y;
    double z;

    std::size_t neighbors;
    double distance;
    double z_diff;

    void create_rclcpp_instances();
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<rclcppCloud>::SharedPtr lidar_pc2_sub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr test_pc2_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr mid360_rotated_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr leaf_down_sampling_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr outlier_removal_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr without_ground_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr clustered_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr clustered_conveyor_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr only_legs_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    
    DownSampling down_sampler;
    OutlierRemoval outlier_remover;
    GroundRemoval ground_remover;
    ClusterExtraction clusteler;
    ClusterExtraction clusteler_conveyor;
};