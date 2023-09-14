#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using CloudPtr = Cloud::Ptr;
using rclcppCloud = sensor_msgs::msg::PointCloud2;
using rclcppCloudSharedPtr = rclcppCloud::SharedPtr;

CloudPtr convert_point_cloud2_to_cloud_ptr(rclcppCloudSharedPtr pc2);
rclcppCloud convert_cloud_ptr_to_point_cloud2(CloudPtr cloud, rclcpp::Node *node);
void print_diffrence(const std::string &logger_name, CloudPtr cloud1, CloudPtr cloud2);
CloudPtr rotate(CloudPtr cloud, double angle);