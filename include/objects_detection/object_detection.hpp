#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <objects_detection/utils.hpp>
#include <objects_detection/ground_removal.hpp>

using namespace std::chrono_literals;

class ObjectDetection : public rclcpp::Node {
public:
    ObjectDetection();

private:
    void create_rclcpp_instances();
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;

    // ulities
    CloudPtr convert_point_cloud2_to_cloud_ptr(rclcppCloudSharedPtr pc2) const;
    rclcppCloud convert_cloud_ptr_to_point_cloud2(CloudPtr cloud) const;

    rclcpp::Subscription<rclcppCloud>::SharedPtr lidar_pc2_sub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr ground_pc2_pub_;
};