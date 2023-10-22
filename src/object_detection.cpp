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

    declare_parameter("general.histogram.resolution", "0.1");
    declare_parameter("general.histogram.min", "15");
    declare_parameter("general.histogram.max", "30");
}

void ObjectDetection::get_parameters() {
    pointcloud_topic_name = get_parameter("general.pointcloud_topic_name").as_string();
    roll = std::stod(get_parameter("general.transform.roll").as_string());
    pitch = std::stod(get_parameter("general.transform.pitch").as_string());
    yaw = std::stod(get_parameter("general.transform.yaw").as_string());

    x = std::stod(get_parameter("general.transform.x").as_string());
    y = std::stod(get_parameter("general.transform.y").as_string());
    z = std::stod(get_parameter("general.transform.z").as_string());

    histogram_resolution = std::stod(get_parameter("general.histogram.resolution").as_string());
    histogram_min = std::stoi(get_parameter("general.histogram.min").as_string());
    histogram_max = std::stoi(get_parameter("general.histogram.max").as_string());
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

    density_histogram_pub_ = create_publisher<sensor_msgs::msg::Image>("inz/desity_histogram", 10);
    density_clustered_histogram_pub_ = create_publisher<sensor_msgs::msg::Image>("inz/desity_clustered_histogram", 10);

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
    leaf_down_sampling_pub_->publish(convert_cloud_ptr_to_point_cloud2(tunneled_cloud, frame, this));

    auto histogram = create_histogram(tunneled_cloud, histogram_resolution);

    auto dencities = count_densities(histogram);
    density_histogram_pub_->publish(create_image_from_histogram(histogram));

    auto clustered_histogram = threshold_histogram(histogram, histogram_min, histogram_max);
    density_clustered_histogram_pub_->publish(create_image_from_histogram(clustered_histogram));

    auto low_density_cloud = filter_with_dencity_on_x_image(tunneled_cloud, clustered_histogram, histogram_resolution);
    without_ground_pub_->publish(convert_cloud_ptr_to_point_cloud2(low_density_cloud, frame, this));

    // save_dencities_to_file(dencities, "/home/rabin/Documents/obsidian/inz/inżynierka/ws/data.txt");

    auto end = std::chrono::high_resolution_clock::now();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    RCLCPP_INFO_STREAM(get_logger(), "Callback Took: " << microseconds / 10e6 << "s");
}

CloudPtr ObjectDetection::remove_points_beyond_tunnel(CloudPtr cloud) {
    CloudPtr new_cloud(new Cloud);
    for (const auto& point : cloud->points) {
        if (std::abs(point.y) < 2.0 and point.z < 2.0) {
            new_cloud->push_back(point);
        }
    }
    new_cloud->width = 1;
    new_cloud->height = new_cloud->size();
    return new_cloud;
}
// 4x2m tunnel size
Histogram ObjectDetection::create_histogram(CloudPtr cloud, double resolution) {
    Histogram histogram_image;
    const auto width = static_cast<std::size_t>(TUNNEL_WIDTH / resolution);
    const auto height = static_cast<std::size_t>(TUNNEL_HEIGHT / resolution);
    histogram_image.resize(height);

    for (auto& column : histogram_image) {
        column.resize(width);
    }

    for (const auto& point : cloud->points) {
        const auto image_width_pos = static_cast<std::size_t>((TUNNEL_WIDTH / 2 + point.y) / resolution);
        const auto image_height_pos = static_cast<std::size_t>(point.z / resolution);

        if (image_width_pos >= width or image_height_pos >= height) {
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

    for (size_t i = 0; i < image_msg.height; ++i) {
        for (size_t j = 0; j < histogram[i].size(); ++j) {
            uint8_t intensity = static_cast<uint8_t>(255 * histogram[i][j] / max_element);
            image_msg.data[(image_msg.height - i - 1) * image_msg.step + j] = intensity;
        }
    }

    return image_msg;
}

std::vector<std::size_t> ObjectDetection::count_densities(const Histogram& histogram) {
    // TODO: another function
    auto max_element = std::numeric_limits<std::size_t>::min();
    for (const auto& col : histogram) {
        const auto col_max = *std::max_element(col.begin(), col.end());
        max_element = col_max > max_element ? col_max : max_element;
    }
    std::vector<std::size_t> densities;
    densities.resize(max_element + 1);
    std::ofstream file("/home/rabin/Documents/obsidian/inz/inżynierka/ws/data.txt");
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file." << std::endl;
        return std::vector<std::size_t>();
    }
    for (const auto& col : histogram) {
        for (const auto& density : col) {
            if (density > 0) file << density << "\n";
            ++densities[density];
        }
    }
    file.close();

    return densities;
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

CloudPtr ObjectDetection::filter_with_dencity_on_x_image(CloudPtr cloud, const Histogram& histogram,
                                                         double resolution) {
    const auto width = static_cast<std::size_t>(TUNNEL_WIDTH / resolution);
    const auto height = static_cast<std::size_t>(TUNNEL_HEIGHT / resolution);

    auto low_density_cloud = CloudPtr(new Cloud);
    for (const auto& point : cloud->points) {
        const auto image_width_pos = static_cast<std::size_t>((TUNNEL_WIDTH / 2 + point.y) / resolution);
        const auto image_height_pos = static_cast<std::size_t>(point.z / resolution);

        if (image_width_pos >= width or image_height_pos >= height) {
            continue;
        }
        if (histogram[image_height_pos][image_width_pos]) {
            low_density_cloud->points.push_back(point);
        }
    }
    low_density_cloud->width = 1;
    low_density_cloud->height = low_density_cloud->size();
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
