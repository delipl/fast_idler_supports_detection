#include <objects_detection/object_detection.hpp>

ObjectDetection::ObjectDetection() : Node("object_detection") {
    declare_parameters();
    get_parameters();
    create_rclcpp_instances();
}

void ObjectDetection::declare_parameters() {
    declare_parameter("general.pointcloud_topic_name", "/velodyne_points");
    declare_parameter("general.rotate_angle", "0.55");

    declare_parameter("down_sampler.leaf_size", "0.05");

    declare_parameter("outlier_remover.statistical.neightbours_count", "50");
    declare_parameter("outlier_remover.statistical.stddev_mul_thresh", "-0.01");

    declare_parameter("outlier_remover.radius_outlier.neightbours_count", "50");
    declare_parameter("outlier_remover.radius_outlier.radius", "1.0");

    declare_parameter("ground_remover.dummy.threshold", "-1.2");
    declare_parameter("ground_remover.planar_segmentation.threshold", "0.15");
    declare_parameter("ground_remover.planar_segmentation.eps_angle", "0.01");
}

void ObjectDetection::get_parameters() {
    pointcloud_topic_name = get_parameter("general.pointcloud_topic_name").as_string();
    rotate_angle = std::stod(get_parameter("general.rotate_angle").as_string());

    down_sampler.leaf_size = std::stod(get_parameter("down_sampler.leaf_size").as_string());

    outlier_remover.statistical_neightbours_count = std::stoi(get_parameter("outlier_remover.statistical.neightbours_count").as_string());
    outlier_remover.statistical_stddev_mul_thresh = std::stod(get_parameter("outlier_remover.statistical.stddev_mul_thresh").as_string());
    outlier_remover.radius_outlier_neightbours_count = std::stoi(get_parameter("outlier_remover.radius_outlier.neightbours_count").as_string());
    outlier_remover.radius_outlier_radius = std::stod(get_parameter("outlier_remover.radius_outlier.radius").as_string());

}

void ObjectDetection::create_rclcpp_instances() {
    test_pc2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inz/test", 10);
    mid360_rotated_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inz/rotated", 10);
    leaf_down_sampling_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inz/down_sampled", 10);
    outlier_removal_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inz/outlier_removal", 10);
    without_ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inz/ground_removal", 10);
    clustered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inz/clustered", 10);
    using std::placeholders::_1;
    const std::string &topic_name = pointcloud_topic_name;
    lidar_pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name, 10, std::bind(&ObjectDetection::lidar_callback, this, _1));

}

void ObjectDetection::lidar_callback(const rclcppCloudSharedPtr msg) {
    get_parameters();
    RCLCPP_INFO(get_logger(), "Pointcloud callback.");
    if (msg->width * msg->height == 0) {
        RCLCPP_WARN(get_logger(), "Empty pointcloud skipping...");
        return;
    }
    auto frame = msg->header.frame_id;
    auto cloud{convert_point_cloud2_to_cloud_ptr(msg)};
    auto transformed_cloud = rotate(cloud, rotate_angle);
    mid360_rotated_pub_->publish(convert_cloud_ptr_to_point_cloud2(transformed_cloud, frame, this));

    
    auto down_sampled = down_sampler.leaf(transformed_cloud);
    leaf_down_sampling_pub_->publish(convert_cloud_ptr_to_point_cloud2(down_sampled, frame, this));

    
    auto removed_outliers = outlier_remover.radius_outlier(down_sampled);
    outlier_removal_pub_->publish(convert_cloud_ptr_to_point_cloud2(removed_outliers, frame, this));

    GroundRemoval ground_remover;
    // auto ground_removed = ground_remover.planar_segmentation(removed_outliers, 0.15, 0.01);
    auto ground_removed = ground_remover.dummy(removed_outliers, -1.2);
    without_ground_pub_->publish(convert_cloud_ptr_to_point_cloud2(ground_removed, frame, this));

    // ClusterExtraction clusteler;
    // auto clustered_clouds = clusteler.euclidean(ground_removed, 0.1, 10, 1000);
    // CloudIPtr merged_clustered_cloud(new CloudI);
    // for (auto &clustered_cloud : clustered_clouds) {
    //     *merged_clustered_cloud += *clustered_cloud;
    // }
    // clustered_pub_->publish(convert_cloudi_ptr_to_point_cloud2(merged_clustered_cloud, frame, this));
}