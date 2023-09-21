#include <objects_detection/utils.hpp>

CloudPtr convert_point_cloud2_to_cloud_ptr(rclcppCloudSharedPtr pc2) {
    pcl::PCLPointCloud2 temp_cloud;
    CloudPtr cloud(new Cloud);
    pcl_conversions::toPCL(*pc2, temp_cloud);
    pcl::fromPCLPointCloud2(temp_cloud, *cloud);
    return cloud;
}

rclcppCloud convert_cloud_ptr_to_point_cloud2(CloudPtr cloud, const std::string& frame_name, rclcpp::Node *node) {
    sensor_msgs::msg::PointCloud2 point_cloud;
    pcl::PCLPointCloud2 point_cloud2;
    pcl::toPCLPointCloud2(*cloud, point_cloud2);
    pcl_conversions::fromPCL(point_cloud2, point_cloud);

    point_cloud.header.frame_id = frame_name;
    point_cloud.header.stamp = node->get_clock()->now();
    point_cloud.is_dense = true;
    return point_cloud;
}

rclcppCloud convert_cloudi_ptr_to_point_cloud2(CloudIPtr cloud, const std::string& frame_name, rclcpp::Node *node){
    sensor_msgs::msg::PointCloud2 point_cloud;
    pcl::PCLPointCloud2 point_cloud2;
    pcl::toPCLPointCloud2(*cloud, point_cloud2);
    pcl_conversions::fromPCL(point_cloud2, point_cloud);

    point_cloud.header.frame_id = frame_name;
    point_cloud.header.stamp = node->get_clock()->now();
    point_cloud.is_dense = true;
    return point_cloud;
}


void print_diffrence(const std::string &logger_name, CloudPtr cloud1, CloudPtr cloud2) {
    const auto removed_points_count = cloud1->points.size() - cloud2->points.size();
    RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name),
                       "Got: " << cloud1->points.size() << " points.");
    RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name),
                       "Removed: " << removed_points_count << " points.");
}

CloudPtr rotate(CloudPtr cloud, double angle) {
    CloudPtr transformed_cloud(new Cloud);
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.rotate (Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitY()));
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform_2);
    return transformed_cloud;
}