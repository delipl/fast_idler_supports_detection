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
    declare_parameter("general.transform.yaw", "-0.06");

    declare_parameter("general.transform.x", "0.0");
    declare_parameter("general.transform.y", "0.0");
    declare_parameter("general.transform.z", "1.35");

    declare_parameter("general.tunnel.height", "1.5");
    declare_parameter("general.tunnel.width", "3.00");
    declare_parameter("general.tunnel.length", "5.00");

    declare_parameter("general.forward.histogram.resolution", "0.1");
    declare_parameter("general.forward.histogram.min", "15");
    declare_parameter("general.forward.histogram.max", "30");
    declare_parameter("general.forward.histogram.column_density_threshold", "30");

    declare_parameter("general.ground.histogram.resolution", "0.1");
    declare_parameter("general.ground.histogram.min", "120");
    declare_parameter("general.ground.histogram.max", "350");
    declare_parameter("general.ground.histogram.a", "3.5");

    declare_parameter("outlier_remover.radius_outlier.neighbors_count", "8");
    declare_parameter("outlier_remover.radius_outlier.radius", "0.1");

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

    tunnel_width = std::stod(get_parameter("general.tunnel.width").as_string());
    tunnel_height = std::stod(get_parameter("general.tunnel.height").as_string());
    tunnel_length = std::stod(get_parameter("general.tunnel.length").as_string());

    forward_resolution = std::stod(get_parameter("general.forward.histogram.resolution").as_string());
    forward_histogram_min = std::stoi(get_parameter("general.forward.histogram.min").as_string());
    forward_histogram_max = std::stoi(get_parameter("general.forward.histogram.max").as_string());
    forward_column_density_threshold =
        std::stoi(get_parameter("general.forward.histogram.column_density_threshold").as_string());

    ground_resolution = std::stod(get_parameter("general.ground.histogram.resolution").as_string());
    ground_histogram_min = std::stoi(get_parameter("general.ground.histogram.min").as_string());
    ground_histogram_max = std::stoi(get_parameter("general.ground.histogram.max").as_string());
    ground_histogram_a = std::stoi(get_parameter("general.ground.histogram.a").as_string());

    outlier_remover.radius_outlier_neighbors_count =
        std::stoi(get_parameter("outlier_remover.radius_outlier.neighbors_count").as_string());
    outlier_remover.radius_outlier_radius =
        std::stod(get_parameter("outlier_remover.radius_outlier.radius").as_string());

    clusteler.euclidean_tolerance = std::stod(get_parameter("clusteler.euclidean.tolerance").as_string());
    clusteler.euclidean_max_size = std::stoi(get_parameter("clusteler.euclidean.max_size").as_string());
    clusteler.euclidean_min_size = std::stoi(get_parameter("clusteler.euclidean.min_size").as_string());
}

void ObjectDetection::create_rclcpp_instances() {
    test_pc2_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/test", 10);
    mid360_rotated_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/rotated", 10);
    tunneled_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/tunneled", 10);
    outlier_removal_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/outlier_removal", 10);
    forward_filtered = create_publisher<sensor_msgs::msg::PointCloud2>("inz/forward_filtered", 10);
    ground_filtered = create_publisher<sensor_msgs::msg::PointCloud2>("inz/ground_filtered", 10);
    clustered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/clustered", 10);
    clustered_conveyor_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/clustered_conveyor", 10);
    only_legs_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/only_legs", 10);

    forward_density_histogram_pub_ = create_publisher<sensor_msgs::msg::Image>("inz/forward_desity_histogram", 10);
    forward_density_clustered_histogram_pub_ =
        create_publisher<sensor_msgs::msg::Image>("inz/forward_desity_clustered_histogram", 10);

    ground_density_histogram_pub_ = create_publisher<sensor_msgs::msg::Image>("inz/ground_desity_histogram", 10);
    ground_density_clustered_histogram_pub_ =
        create_publisher<sensor_msgs::msg::Image>("inz/ground_desity_clustered_histogram", 10);
    ground_density_histogram_multiplied_pub_ = 
        create_publisher<sensor_msgs::msg::Image>("inz/ground_desity_multiplied_histogram", 10);
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
    auto tunneled_cloud = remove_points_beyond_tunnel(transformed_cloud);
    tunneled_pub_->publish(convert_cloud_ptr_to_point_cloud2(tunneled_cloud, frame, this));

    auto histogram = create_histogram(tunneled_cloud, forward_resolution, tunnel_width, tunnel_height);
    forward_density_histogram_pub_->publish(create_image_from_histogram(histogram));

    auto clustered_histogram = threshold_histogram(histogram, forward_histogram_min, forward_histogram_max);
    auto removed_columns = remove_low_density_colums(clustered_histogram, forward_column_density_threshold);
    forward_density_clustered_histogram_pub_->publish(create_image_from_histogram(removed_columns));

    auto low_density_cloud = filter_with_dencity_on_x_image(tunneled_cloud, removed_columns, forward_resolution,
                                                            tunnel_width, tunnel_height);

    forward_filtered->publish(convert_cloud_ptr_to_point_cloud2(low_density_cloud, frame, this));

    auto rotated_for_ground_histogram = rotate(tunneled_cloud, 0.0, -M_PI / 2.0, 0.0);
    auto ground_histogram =
        create_histogram(rotated_for_ground_histogram, ground_resolution, tunnel_width, tunnel_length);
    // auto dencities = save_histogram_to_file(histogram);
    ground_density_histogram_pub_->publish(create_image_from_histogram(ground_histogram));
    auto multiplied_ground_histogram = multiply_histogram_by_exp(ground_histogram, ground_histogram_a);
    ground_density_histogram_multiplied_pub_->publish(create_image_from_histogram(multiplied_ground_histogram));

    auto clustered_ground_histogram = threshold_histogram(multiplied_ground_histogram, ground_histogram_min, ground_histogram_max);
    ground_density_clustered_histogram_pub_->publish(create_image_from_histogram(clustered_ground_histogram));
    auto high_density_top_cloud = filter_with_dencity_on_z_image(tunneled_cloud, clustered_ground_histogram, ground_resolution,
                                                            tunnel_width, tunnel_length);

    ground_filtered->publish(convert_cloud_ptr_to_point_cloud2(high_density_top_cloud, frame, this));


    CloudIPtrs clustered_clouds = clusteler.euclidean(high_density_top_cloud);
    max_detected_legs = std::max(max_detected_legs, clustered_clouds.size());
    CloudIPtr merged_clustered_cloud(new CloudI);
    for (auto& clustered_cloud : clustered_clouds) {
        if (clustered_cloud->size()) {
            *merged_clustered_cloud += *clustered_cloud;
        }
    }
    clustered_conveyor_pub_->publish(convert_cloudi_ptr_to_point_cloud2(merged_clustered_cloud, frame, this));

    auto markers = make_markers_from_pointclouds(clustered_clouds);
    markers_pub_->publish(*markers);
    auto end = std::chrono::high_resolution_clock::now();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    RCLCPP_INFO_STREAM(get_logger(), "Callback Took: " << microseconds / 10e6 << "s");
    ++counter;
}

CloudPtr ObjectDetection::remove_points_beyond_tunnel(CloudPtr cloud) {
    CloudPtr new_cloud(new Cloud);
    for (const auto& point : cloud->points) {
        if (std::abs(point.y) < tunnel_width - 2.0 and point.z < tunnel_height) {
            new_cloud->push_back(point);
        }
    }
    new_cloud->width = 1;
    new_cloud->height = new_cloud->size();
    return new_cloud;
}

Histogram ObjectDetection::create_histogram(CloudPtr cloud, double resolution, double width, double height) {
    Histogram histogram_image;
    const auto image_width = static_cast<std::size_t>(width / resolution);
    const auto image_height = static_cast<std::size_t>(height / resolution);
    histogram_image.resize(image_height);

    for (auto& column : histogram_image) {
        column.resize(image_width);
    }

    for (const auto& point : cloud->points) {
        const auto image_width_pos = static_cast<std::size_t>((tunnel_width / 2 + point.y) / resolution);
        const auto image_height_pos = static_cast<std::size_t>(point.z / resolution);

        if (image_width_pos >= image_width or image_height_pos >= image_height) {
            continue;
        }

        ++histogram_image[image_height_pos][image_width_pos];
    }
    return histogram_image;
}

sensor_msgs::msg::Image ObjectDetection::create_image_from_histogram(const Histogram& histogram) {
    sensor_msgs::msg::Image image_msg;
    image_msg.height = histogram.size();
    image_msg.width = histogram.begin()->size();
    image_msg.encoding = "mono8";

    image_msg.step = image_msg.width;
    image_msg.data.resize(image_msg.height * image_msg.step, 0);
    // TODO: another function
    auto max_element = std::numeric_limits<std::size_t>::min();
    for (const auto& col : histogram) {
        const auto col_max = *std::max_element(col.begin(), col.end());
        max_element = std::max(col_max, max_element);
    }
    if (not max_element) {
        return image_msg;
    }

    for (std::size_t i = 0; i < image_msg.height; ++i) {
        for (std::size_t j = 0; j < histogram[i].size(); ++j) {
            uint8_t intensity = static_cast<uint8_t>(255 * histogram[i][j] / max_element);
            image_msg.data[(image_msg.height - i - 1) * image_msg.step + j] = intensity;
        }
    }

    return image_msg;
}

void ObjectDetection::save_histogram_to_file(const Histogram& histogram, const std::string & file_name) {
    std::ofstream file("/home/rabin/Documents/obsidian/inz/inżynierka/ws/to_histogram_pcds/dencities            /dencities_" + file_name + ".txt");
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file." << std::endl;
        return;
    }
    for (const auto& col : histogram) {
        for (const auto& density : col) {
            if (density > 0) file << density << "\n";
        }
    }
    file.close();
}

void ObjectDetection::save_dencities_to_file(const std::vector<std::size_t>& dencities, const std::string& path) {
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file." << std::endl;
        return;
    }

    for (const auto& d : dencities) {
        file << d << "\n";
    }
    file.close();
}

CloudPtr ObjectDetection::filter_with_dencity_on_x_image(CloudPtr cloud, const Histogram& histogram, double resolution,
                                                         double width, double height) {
    const auto image_width = static_cast<std::size_t>(width / resolution);
    const auto image_height = static_cast<std::size_t>(height / resolution);

    auto low_density_cloud = CloudPtr(new Cloud);
    for (const auto& point : cloud->points) {
        const auto image_width_pos = static_cast<std::size_t>((width / 2 + point.y) / resolution);
        const auto image_height_pos = static_cast<std::size_t>(point.z / resolution);

        if (image_width_pos >= image_width or image_height_pos >= image_height) {
            continue;
        }
        if (histogram[image_height_pos][image_width_pos]) {
            low_density_cloud->points.push_back(point);
        }
    }
    low_density_cloud->width = low_density_cloud->size();
    low_density_cloud->height = 1;
    return low_density_cloud;
}

CloudPtr ObjectDetection::filter_with_dencity_on_z_image(CloudPtr cloud, const Histogram& histogram, double resolution,
                                                         double width, double length) {
    const auto image_width = static_cast<std::size_t>(width / resolution);
    const auto image_height = static_cast<std::size_t>(length / resolution);

    auto low_density_cloud = CloudPtr(new Cloud);
    for (const auto& point : cloud->points) {
        const auto image_width_pos = static_cast<std::size_t>((width / 2 + point.y) / resolution);
        const auto image_lenght_pos = static_cast<std::size_t>(point.x / resolution);

        if (image_width_pos >= image_width or image_lenght_pos >= image_height) {
            continue;
        }
        if (histogram[image_lenght_pos][image_width_pos]) {
            low_density_cloud->points.push_back(point);
        }
    }
    low_density_cloud->width = low_density_cloud->size();
    low_density_cloud->height = 1;
    return low_density_cloud;
}

Histogram ObjectDetection::threshold_histogram(const Histogram& histogram, std::size_t min, std::size_t max) {
    auto clustered_histogram(histogram);
    for (auto& col : clustered_histogram) {
        for (auto& density : col) {
            auto clamped_density = std::clamp(density, min, max);
            if (density != clamped_density and density != 0) {
                density = 0;
            }
        }
    }
    return clustered_histogram;
}

Histogram ObjectDetection::remove_low_density_colums(const Histogram& histogram, std::size_t threshold) {
    auto clustered_histogram(histogram);

    for (std::size_t i = 0; i < clustered_histogram[0].size(); ++i) {
        std::size_t sum = 0;
        for (std::size_t j = 0; j < clustered_histogram.size(); ++j) {
            sum += clustered_histogram[j][i];
        }
        if (sum < threshold) {
            for (std::size_t j = 0; j < clustered_histogram.size(); ++j) {
                clustered_histogram[j][i] = 0;
            }
        }
        // RCLCPP_INFO_STREAM(get_logger(), "Column i: " << i << " has sum: " << sum);
    }
    return clustered_histogram;
}

MarkersPtr ObjectDetection::make_markers_from_pointclouds(const CloudIPtrs& clustered_clouds) {
    auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();
    std::size_t count_ = 0;
    for (const auto& leg : clustered_clouds) {
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
        marker.lifetime = rclcpp::Duration(0, 2000);
        count_++;
        marker_array_msg->markers.push_back(marker);
    }
    for (std::size_t i = clustered_clouds.size() - 1; i < max_detected_legs; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "velodyne";
        marker.ns = "cylinders";
        marker.id = i;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array_msg->markers.push_back(marker);
    }
    return marker_array_msg;
}

Histogram ObjectDetection::multiply_histogram_by_exp(const Histogram& histogram, double a) {
    Histogram multipied_histogram(histogram);
    for (auto i = 0u; i < multipied_histogram.size(); ++i) {
        for (auto j = 0u; j < multipied_histogram[i].size(); ++j) {
            multipied_histogram[i][j] *= std::exp(a*i/multipied_histogram.size());
        }
    }
    return multipied_histogram;
}
