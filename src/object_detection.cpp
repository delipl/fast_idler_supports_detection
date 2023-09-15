#include <objects_detection/object_detection.hpp>

ObjectDetection::ObjectDetection() : Node("object_detection") { create_rclcpp_instances(); }

void ObjectDetection::create_rclcpp_instances() {
    test_pc2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inz/test", 10);
    mid360_rotated_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inz/rotated", 10);
    leaf_down_sampling_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inz/down_sampled", 10);
    outlier_removal_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inz/outlier_removal", 10);
    without_ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inz/ground_removal", 10);
    clustered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inz/clustered", 10);
    using std::placeholders::_1;
    const std::string &topic_name = "/mid360";
    // const std::string &topic_name = "/livox/lidar";
    lidar_pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name, 10, std::bind(&ObjectDetection::lidar_callback, this, _1));
}

void ObjectDetection::lidar_callback(const rclcppCloudSharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Pointcloud callback.");
    if (msg->width * msg->height == 0) {
        RCLCPP_WARN(this->get_logger(), "Empty pointcloud skipping...");
        return;
    }

    auto cloud{convert_point_cloud2_to_cloud_ptr(msg)};
    auto transformed_cloud = rotate(cloud, 0.15);
    mid360_rotated_pub_->publish(convert_cloud_ptr_to_point_cloud2(transformed_cloud, this));

    DownSampling down_sampler;
    auto down_sampled = down_sampler.leaf(transformed_cloud, 0.05, 0.05, 0.05);
    leaf_down_sampling_pub_->publish(convert_cloud_ptr_to_point_cloud2(down_sampled, this));

    OutlierRemoval remover;
    auto removed_outliers = remover.statistical_pcl(down_sampled, 50, 0.01);
    outlier_removal_pub_->publish(convert_cloud_ptr_to_point_cloud2(removed_outliers, this));

    GroundRemoval algorythm;
    auto ground_removed = algorythm.planar_segmentation(removed_outliers, 0.15, 0.01);
    without_ground_pub_->publish(convert_cloud_ptr_to_point_cloud2(ground_removed, this));

    ClusterExtraction clusteler;
    auto clustered_clouds = clusteler.euclidean(ground_removed, 0.1, 10, 1000);
    CloudIPtr merged_clustered_cloud(new CloudI);
    for (auto &clustered_cloud : clustered_clouds) {
        *merged_clustered_cloud += *clustered_cloud;
    }
    clustered_pub_->publish(convert_cloudi_ptr_to_point_cloud2(merged_clustered_cloud, this));
}