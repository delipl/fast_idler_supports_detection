#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using CloudPtr = Cloud::Ptr;
using rclcppCloud = sensor_msgs::msg::PointCloud2;
using rclcppCloudSharedPtr = rclcppCloud::SharedPtr;