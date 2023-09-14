#include <objects_detection/object_detection.hpp>

ObjectDetection::ObjectDetection() : Node("object_detection") { create_rclcpp_instances(); }

void ObjectDetection::create_rclcpp_instances() {
    test_pc2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inz/test", 10);
    rotated_without_ground_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("inz/rotated_without_ground", 10);

    using std::placeholders::_1;
    const std::string &topic_name = "/mid360";
    // std::string topic_name = "/livox/lidar";
    lidar_pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name, 10, std::bind(&ObjectDetection::lidar_callback, this, _1));
}
static std::size_t cnt = 0;
void ObjectDetection::lidar_callback(const rclcppCloudSharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Pointcloud callback.");
    if (msg->width * msg->height == 0) {
        RCLCPP_WARN(this->get_logger(), "Empty pointcloud skipping...");
        return;
    }

    auto cloud{convert_point_cloud2_to_cloud_ptr(msg)};


    cnt++;

    auto transformed_cloud = rotate(cloud, 0.15);
    test_pc2_pub_->publish(convert_cloud_ptr_to_point_cloud2(transformed_cloud, this));

    GroundRemoval algorythm;
    auto cloud_test = algorythm.planar_segmentation(transformed_cloud, 0.15, 0.01);
    rotated_without_ground_pub_->publish(convert_cloud_ptr_to_point_cloud2(cloud_test, this));
}