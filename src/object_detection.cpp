#include <objects_detection/object_detection.hpp>

ObjectDetection::ObjectDetection() : Node("object_detection") { create_rclcpp_instances(); }

void ObjectDetection::create_rclcpp_instances() {
    test_pc2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inz/test", 10);

    using std::placeholders::_1;
    lidar_pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mid360", 10, std::bind(&ObjectDetection::lidar_callback, this, _1));
}
static std::size_t cnt = 0;
void ObjectDetection::lidar_callback(const rclcppCloudSharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Pointcloud callback.");
    if (msg->width * msg->height == 0) {
        RCLCPP_WARN(this->get_logger(), "Empty pointcloud skipping...");
        return;
    }

    auto cloud{convert_point_cloud2_to_cloud_ptr(msg)};
    OutlierRemoval algorythm;
    pcl::PCDWriter writer;
    std::string path = "/home/rabin/Documents/obsidian/lol/inżynierka/ws/src/objects_detection/test_data/test_cloud.pcd";
    writer.write<pcl::PointXYZ>(path, *cloud);
    auto clound_test = algorythm.statistical_pcl(cloud, 20, -0.01);

    cnt ++;

    test_pc2_pub_->publish(convert_cloud_ptr_to_point_cloud2(clound_test, this));
}