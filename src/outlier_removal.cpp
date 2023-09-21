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

    print_diffrence("statistical_outlier_removal", cloud, filtered);

    return filtered;
}

CloudPtr OutlierRemoval::statistical_pcl(CloudPtr cloud) const{
    return statistical_pcl(cloud, neightbours_count, stddev_mul_thresh);
}