#include <iostream>
#include <objects_detection/outlier_removal.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

CloudPtr OutlierRemoval::statistical_pcl(CloudPtr cloud, uint k, double stddev_mul_thresh) const {
    pcl::StatisticalOutlierRemoval<Point> filter;
    CloudPtr filtered(new Cloud);
    filter.setInputCloud(cloud);
    filter.setMeanK(k);
    filter.setStddevMulThresh(stddev_mul_thresh);
    filter.filter(*filtered);
    const auto removed_points_count = cloud->points.size() - filtered->points.size();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("statistical_outlier_removal"),
                       "Got: " << cloud->points.size() << " points.");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("statistical_outlier_removal"),
                       "Removed: " << removed_points_count << " points.");
    return filtered;
}