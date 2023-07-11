#include <objects_detection/object_detection.hpp>

ObjectDetection::ObjectDetection() : Node("object_detection") {
    create_rclcpp_instances();
}

void ObjectDetection::create_rclcpp_instances() {
    test_pc2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inz/test", 10);

    using std::placeholders::_1;
    lidar_pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/mid360", 10,
        std::bind(&ObjectDetection::lidar_callback, this, _1));

}

CloudPtr ObjectDetection::convert_point_cloud2_to_cloud_ptr(rclcppCloudSharedPtr pc2) const {
    pcl::PCLPointCloud2 temp_cloud;
    CloudPtr cloud(new Cloud);
    pcl_conversions::toPCL(*pc2, temp_cloud);
    pcl::fromPCLPointCloud2(temp_cloud, *cloud);
    return cloud;
}

rclcppCloud ObjectDetection::convert_cloud_ptr_to_point_cloud2(CloudPtr cloud) const {
    sensor_msgs::msg::PointCloud2 point_cloud;
    pcl::PCLPointCloud2 point_cloud2;
    pcl::toPCLPointCloud2(*cloud, point_cloud2);
    pcl_conversions::fromPCL(point_cloud2, point_cloud);

    point_cloud.header.frame_id = "livox";
    point_cloud.header.stamp = rclcpp::Node::now();
    point_cloud.is_dense = true;
    return point_cloud;
}

void ObjectDetection::lidar_callback(const rclcppCloudSharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "Pointcloud callback.");
    if (msg->width * msg->height == 0) {
        RCLCPP_WARN(this->get_logger(), "Empty pointcloud skipping...");
        return;
    }

    auto cloud{ convert_point_cloud2_to_cloud_ptr(msg) };
    OutlierRemoval algorythm;
    auto clound_test = algorythm.statistical_pcl(cloud, 20, -0.01);
    test_pc2_pub_->publish(convert_cloud_ptr_to_point_cloud2(clound_test));
}