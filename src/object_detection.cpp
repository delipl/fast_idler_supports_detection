#include <objects_detection/object_detection.hpp>

ObjectDetection::ObjectDetection() : Node("object_detection") {
    declare_parameters();
    get_parameters();
    create_rclcpp_instances();
}

void ObjectDetection::declare_parameters() {
    // Double parameters are strings to work with rqt dynamic reconfigure.
    declare_parameter("general.pointcloud_topic_name", "/velodyne_points");
    declare_parameter("general.transform.roll", "-0.09");
    declare_parameter("general.transform.pitch", "0.55");
    declare_parameter("general.transform.yaw", "0.0");

    declare_parameter("general.transform.x", "0.0");
    declare_parameter("general.transform.y", "0.0");
    declare_parameter("general.transform.z", "1.35");

    declare_parameter("down_sampler.leaf_size", "0.05");

    declare_parameter("outlier_remover.statistical.neighbors_count", "50");
    declare_parameter("outlier_remover.statistical.stddev_mul_thresh", "-0.01");

    declare_parameter("outlier_remover.radius_outlier.neighbors_count", "8");
    declare_parameter("outlier_remover.radius_outlier.radius", "0.1");

    declare_parameter("outlier_remover.radius_vertical_outlier.neighbors_count", "8");
    declare_parameter("outlier_remover.radius_vertical_outlier.radius", "0.1");
    declare_parameter("outlier_remover.radius_vertical_outlier.diff", "0.07");

    declare_parameter("ground_remover.dummy.threshold", "0.1");
    declare_parameter("ground_remover.planar_segmentation.threshold", "0.15");
    declare_parameter("ground_remover.planar_segmentation.eps_angle", "0.01");

    declare_parameter("clusteler_conveyor.euclidean.tolerance", "0.5");
    declare_parameter("clusteler_conveyor.euclidean.min_size", "1000");
    declare_parameter("clusteler_conveyor.euclidean.max_size", "5000");

    declare_parameter("clusteler.euclidean.tolerance", "0.5");
    declare_parameter("clusteler.euclidean.min_size", "20");
    declare_parameter("clusteler.euclidean.max_size", "500");
}

void ObjectDetection::get_parameters() {
    pointcloud_topic_name = get_parameter("general.pointcloud_topic_name").as_string();
    roll = std::stod(get_parameter("general.transform.roll").as_string());
    pitch = std::stod(get_parameter("general.transform.pitch").as_string());
    yaw = std::stod(get_parameter("general.transform.yaw").as_string());

    x = std::stod(get_parameter("general.transform.x").as_string());
    y = std::stod(get_parameter("general.transform.y").as_string());
    z = std::stod(get_parameter("general.transform.z").as_string());

    down_sampler.leaf_size = std::stod(get_parameter("down_sampler.leaf_size").as_string());

    outlier_remover.statistical_neighbors_count =
        std::stoi(get_parameter("outlier_remover.statistical.neighbors_count").as_string());
    outlier_remover.statistical_stddev_mul_thresh =
        std::stod(get_parameter("outlier_remover.statistical.stddev_mul_thresh").as_string());
    outlier_remover.radius_outlier_neighbors_count =
        std::stoi(get_parameter("outlier_remover.radius_outlier.neighbors_count").as_string());
    outlier_remover.radius_outlier_radius =
        std::stod(get_parameter("outlier_remover.radius_outlier.radius").as_string());

    outlier_remover.radius_vertical_outlier_neighbors_count =
        std::stoi(get_parameter("outlier_remover.radius_vertical_outlier.neighbors_count").as_string());
    outlier_remover.radius_vertical_outlier_radius =
        std::stod(get_parameter("outlier_remover.radius_vertical_outlier.radius").as_string());
    outlier_remover.radius_vertical_outlier_diff =
        std::stod(get_parameter("outlier_remover.radius_vertical_outlier.diff").as_string());

    ground_remover.dummy_threshold = std::stod(get_parameter("ground_remover.dummy.threshold").as_string());
    ground_remover.planar_segmentation_threshold =
        std::stod(get_parameter("ground_remover.planar_segmentation.threshold").as_string());
    ground_remover.planar_segmentation_eps_angle =
        std::stod(get_parameter("ground_remover.planar_segmentation.eps_angle").as_string());

    clusteler.euclidean_tolerance = std::stod(get_parameter("clusteler.euclidean.tolerance").as_string());
    clusteler.euclidean_max_size = std::stoi(get_parameter("clusteler.euclidean.max_size").as_string());
    clusteler.euclidean_min_size = std::stoi(get_parameter("clusteler.euclidean.min_size").as_string());

    clusteler_conveyor.euclidean_tolerance =
        std::stod(get_parameter("clusteler_conveyor.euclidean.tolerance").as_string());
    clusteler_conveyor.euclidean_max_size =
        std::stoi(get_parameter("clusteler_conveyor.euclidean.max_size").as_string());
    clusteler_conveyor.euclidean_min_size =
        std::stoi(get_parameter("clusteler_conveyor.euclidean.min_size").as_string());
}

void ObjectDetection::create_rclcpp_instances() {
    test_pc2_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/test", 10);
    mid360_rotated_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/rotated", 10);
    leaf_down_sampling_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/down_sampled", 10);
    outlier_removal_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/outlier_removal", 10);
    without_ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/ground_removal", 10);
    clustered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/clustered", 10);
    clustered_conveyor_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/clustered_conveyor", 10);
    only_legs_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/only_legs", 10);

    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("marker_array", 10);

    using std::placeholders::_1;
    const std::string& topic_name = pointcloud_topic_name;
    lidar_pc2_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name, 10, std::bind(&ObjectDetection::lidar_callback, this, _1));
}

void ObjectDetection::lidar_callback(const rclcppCloudSharedPtr msg) {
    auto start = std::chrono::high_resolution_clock::now();
    get_parameters();
    RCLCPP_INFO(get_logger(), "Pointcloud callback.");
    if (msg->width * msg->height == 0) {
        RCLCPP_WARN(get_logger(), "Empty pointcloud skipping...");
        return;
    }
    auto frame = msg->header.frame_id;
    auto cloud{convert_point_cloud2_to_cloud_ptr(msg)};
    auto transformed_cloud = translate(rotate(cloud, roll, pitch, yaw), x, y, z);

    mid360_rotated_pub_->publish(convert_cloud_ptr_to_point_cloud2(transformed_cloud, frame, this));
    auto removed_outliers = outlier_remover.radius_outlier(transformed_cloud);
    outlier_removal_pub_->publish(convert_cloud_ptr_to_point_cloud2(removed_outliers, frame, this));

    auto ground_removed = ground_remover.dummy(removed_outliers);
    without_ground_pub_->publish(convert_cloud_ptr_to_point_cloud2(ground_removed, frame, this));

    CloudIPtrs clustered_clouds = clusteler_conveyor.euclidean(ground_removed);
    CloudIPtr merged_clustered_cloud(new CloudI);
    for (auto& clustered_cloud : clustered_clouds) {
        if (clustered_cloud->size()) {
            *merged_clustered_cloud += *clustered_cloud;
        }
    }
    clustered_conveyor_pub_->publish(convert_cloudi_ptr_to_point_cloud2(merged_clustered_cloud, frame, this));
    auto merged_clustered_cloud_no_color = remove_intensivity_from_cloud(merged_clustered_cloud);
    auto verical_points = outlier_remover.radius_vertical_outlier(merged_clustered_cloud_no_color);
    only_legs_pub_->publish(convert_cloud_ptr_to_point_cloud2(verical_points, frame, this));

    CloudIPtrs legs = clusteler.euclidean(verical_points);

    CloudIPtr merged_legs(new CloudI);
    for (auto& clustered_cloud : legs) {
        if (clustered_cloud->size()) {
            *merged_legs += *clustered_cloud;
        }
    }
    clustered_pub_->publish(convert_cloudi_ptr_to_point_cloud2(merged_legs, frame, this));

    auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();
    std::size_t count_ = 0;
    for (const auto& leg : legs) {
        Point mean{0.0, 0.0, 0.0};
        for (const auto& point : leg->points) {
            mean.x += point.x;
            mean.y += point.y;
            mean.z += point.z;
        }
        const auto& size = leg->points.size();
        mean.x /= size;
        mean.y /= size;
        mean.z /= size;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "velodyne";
        marker.ns = "cylinders";
        marker.id = count_;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = mean.x;
        marker.pose.position.y = mean.y;
        marker.pose.position.z = mean.z;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.5;  // Height of the cylinder
        marker.color.a = 1.0;  // Alpha
        marker.color.r = 0.0;  // Red
        marker.color.g = 0.0;  // Green
        marker.color.b = 1.0;  // Blue
        count_++;
        marker_array_msg->markers.push_back(marker);
    }
    if (marker_array_msg->markers.size()) {
        marker_array_msg->markers.front().action = visualization_msgs::msg::Marker::DELETEALL;
        markers_pub_->publish(*marker_array_msg);
        marker_array_msg->markers.front().action = visualization_msgs::msg::Marker::ADD;
        markers_pub_->publish(*marker_array_msg);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    RCLCPP_INFO_STREAM(get_logger(), "Callback Took: " << microseconds / 10e6 << "s");
}