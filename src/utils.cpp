#include <objects_detection/utils.hpp>

CloudPtr convert_point_cloud2_to_cloud_ptr(rclcppCloudSharedPtr pc2) {
    pcl::PCLPointCloud2 temp_cloud;
    CloudPtr cloud(new Cloud);
    pcl_conversions::toPCL(*pc2, temp_cloud);
    pcl::fromPCLPointCloud2(temp_cloud, *cloud);
    return cloud;
}

rclcppCloud convert_cloud_ptr_to_point_cloud2(CloudPtr cloud, rclcpp::Node *node) {
    sensor_msgs::msg::PointCloud2 point_cloud;
    pcl::PCLPointCloud2 point_cloud2;
    pcl::toPCLPointCloud2(*cloud, point_cloud2);
    pcl_conversions::fromPCL(point_cloud2, point_cloud);

    point_cloud.header.frame_id = "livox";
    point_cloud.header.stamp = node->get_clock()->now();
    point_cloud.is_dense = true;
    return point_cloud;
}