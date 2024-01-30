#include <fstream>
#include <objects_detection/object_detection.hpp>

ObjectDetection::ObjectDetection() : Node("object_detection") {
    declare_parameters();
    get_parameters();
    create_rclcpp_instances();
    std::ofstream file(filename);
}

void ObjectDetection::declare_parameters() {
    // Double parameters are strings to work with rqt dynamic reconfigure.
    declare_parameter("general.filename", "output.yaml");

    declare_parameter("general.pointcloud_topic_name", "/velodyne_points");
    declare_parameter("general.transform.roll", "-0.09");
    declare_parameter("general.transform.pitch", "0.55");
    declare_parameter("general.transform.yaw", "-0.06");
    declare_parameter("general.ground_level_height", "0.07");

    declare_parameter("general.transform.x", "0.0");
    declare_parameter("general.transform.y", "0.0");
    declare_parameter("general.transform.z", "1.35");

    declare_parameter("general.tunnel.height", "1.5");
    declare_parameter("general.tunnel.width", "5.00");
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

    declare_parameter("conveyor_candidates_clusteler.euclidean.tolerance", "0.5");
    declare_parameter("conveyor_candidates_clusteler.euclidean.min_size", "100");
    declare_parameter("conveyor_candidates_clusteler.euclidean.max_size", "3000");

    declare_parameter("clusteler.euclidean.tolerance", "0.5");
    declare_parameter("clusteler.euclidean.min_size", "20");
    declare_parameter("clusteler.euclidean.max_size", "500");
}

void ObjectDetection::get_parameters() {
    filename = get_parameter("general.filename").as_string();
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
    ground_level_height = std::stod(get_parameter("general.ground_level_height").as_string());

    forward_resolution = std::stod(get_parameter("general.forward.histogram.resolution").as_string());
    forward_histogram_min = std::stoi(get_parameter("general.forward.histogram.min").as_string());
    forward_histogram_max = std::stoi(get_parameter("general.forward.histogram.max").as_string());
    forward_column_density_threshold =
        std::stoi(get_parameter("general.forward.histogram.column_density_threshold").as_string());

    ground_resolution = std::stod(get_parameter("general.ground.histogram.resolution").as_string());
    ground_histogram_min = std::stoi(get_parameter("general.ground.histogram.min").as_string());
    ground_histogram_max = std::stoi(get_parameter("general.ground.histogram.max").as_string());
    ground_histogram_a = std::stoi(get_parameter("general.ground.histogram.a").as_string());

    conveyor_candidates_clusteler.euclidean_tolerance =
        std::stod(get_parameter("conveyor_candidates_clusteler.euclidean.tolerance").as_string());
    conveyor_candidates_clusteler.euclidean_max_size =
        std::stoi(get_parameter("conveyor_candidates_clusteler.euclidean.max_size").as_string());
    conveyor_candidates_clusteler.euclidean_min_size =
        std::stoi(get_parameter("conveyor_candidates_clusteler.euclidean.min_size").as_string());

    clusteler.euclidean_tolerance = std::stod(get_parameter("clusteler.euclidean.tolerance").as_string());
    clusteler.euclidean_max_size = std::stoi(get_parameter("clusteler.euclidean.max_size").as_string());
    clusteler.euclidean_min_size = std::stoi(get_parameter("clusteler.euclidean.min_size").as_string());
}

void ObjectDetection::create_rclcpp_instances() {
    transformed_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/rotated", 10);
    tunneled_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/tunneled", 10);
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/ground_pub_", 10);
    without_ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/without_ground_pub_", 10);
    outlier_removal_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/outlier_removal", 10);
    forward_hist_filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/forward_hist_filtered_pub_", 10);
    top_hist_filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/top_hist_filtered_pub_", 10);
    clustered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/clustered", 10);
    clustered_conveyor_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/clustered_conveyor", 10);
    clustered_conveyors_candidates_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("inz/clustered_conveyors_candidates", 10);
    merged_density_clouds_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/merged_density_clouds", 10);
    plane_filter_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/plane_filter", 10);

    only_legs_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/only_legs", 10);
    bounding_box_pub_ = create_publisher<vision_msgs::msg::BoundingBox3DArray>("inz/bounding_boxes", 10);
    detection_3d_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>("inz/detected_conveyors", 10);
    conveyors_candidates_bounding_box_pub_ =
        create_publisher<vision_msgs::msg::BoundingBox3DArray>("inz/conveyor_candidates_bounding_boxes", 10);

    conveyors_bounding_box_pub_ =
        create_publisher<vision_msgs::msg::BoundingBox3DArray>("inz/conveyor_bounding_boxes", 10);
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
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name, 10, std::bind(&ObjectDetection::lidar_callback, this, _1));
}

void ObjectDetection::lidar_callback(const rclcppCloudSharedPtr msg) {
    auto start = std::chrono::high_resolution_clock::now();
    get_parameters();
    RCLCPP_DEBUG(get_logger(), "Pointcloud callback.");
    if (msg->width * msg->height == 0) {
        RCLCPP_WARN(get_logger(), "Empty pointcloud skipping...");
        
        return;
    }

    auto normalization_start = std::chrono::high_resolution_clock::now();
    auto frame = msg->header.frame_id;
    auto cloud_raw{convert_point_cloud2_to_cloud_ptr(msg)};
    auto raw_without_convert_error = remove_far_points_from_ros2bag_converter_bug(cloud_raw, 999.0);
    original_points_count = raw_without_convert_error->size();
    auto cloud = remove_far_points_from_ros2bag_converter_bug(cloud_raw, 5.0);
    filter_further_than_5m_points_count = cloud->size();
    // Ground Filtering
    Eigen::Vector3d normal_vec;
    double ground_height;

    CloudPtr cloud_for_ground_detection(new Cloud);
    for (const auto& point : cloud->points) {
        if (std::abs(point.y) < tunnel_width / 2.0) {
            cloud_for_ground_detection->push_back(point);
        }
    }
    auto filtered_ground_clouds =
        filter_ground_and_get_normal_and_height(cloud_for_ground_detection, pcl::SACMODEL_PLANE, 800,
                                                ground_level_height, std::ref(normal_vec), std::ref(ground_height));
    auto ground = filtered_ground_clouds.first;
    auto without_ground = filtered_ground_clouds.second;
    filter_ground_points_count = without_ground->size();

    auto aligned_cloud = align_to_normal(without_ground, normal_vec, ground_height);

    auto normalization_end = std::chrono::high_resolution_clock::now();

    normalization_duration_count =
        std::chrono::duration_cast<std::chrono::microseconds>(normalization_end - normalization_start).count();
    ground_pub_->publish(convert_cloud_ptr_to_point_cloud2(ground, frame, this));
    without_ground_pub_->publish(convert_cloud_ptr_to_point_cloud2(without_ground, frame, this));
    transformed_pub_->publish(convert_cloud_ptr_to_point_cloud2(aligned_cloud, frame, this));

    // Conveyors detection based on position.z and height
    CloudIPtrs clustered_conveyors_candidates = conveyor_candidates_clusteler.euclidean(aligned_cloud);
    if (not clustered_conveyors_candidates.size()) {
        RCLCPP_WARN(get_logger(), "Cannot find any conveyor candidate! Skipping pointcloud");
        clear_markers(frame);
        return;
    }

    auto conveyors_candidates_detection_3d_msg = detect_conveyors(clustered_conveyors_candidates, frame);

    Detection3DArrayPtr conveyors_detection_3d_msg;
    conveyors_detection_3d_msg =
        std::make_shared<vision_msgs::msg::Detection3DArray>(*conveyors_candidates_detection_3d_msg);
    CloudIPtrs clustered_conveyors(clustered_conveyors_candidates);
    auto it1 = conveyors_detection_3d_msg->detections.begin();
    auto it2 = clustered_conveyors.begin();

    while (it1 != conveyors_detection_3d_msg->detections.end() && it2 != clustered_conveyors.end()) {
        if (it1->results[0].hypothesis.score < 0.5) {
            it1 = conveyors_detection_3d_msg->detections.erase(it1);
            it2 = clustered_conveyors.erase(it2);
        } else {
            ++it1;
            ++it2;
        }
    }

    if (not clustered_conveyors.size()) {
        RCLCPP_WARN(get_logger(), "Cannot find any conveyor! Skipping pointcloud");
        clear_markers(frame);
        return;
    }

    auto merged_conveyors_candidates = merge_clouds(clustered_conveyors_candidates);
    auto merged_conveyors = merge_clouds(clustered_conveyors);
    auto merged_conveyors_without_intensity = remove_intensity_from_cloud(merged_conveyors);

    clustered_conveyors_candidates_pub_->publish(
        convert_cloudi_ptr_to_point_cloud2(merged_conveyors_candidates, frame, this));

    clustered_conveyor_pub_->publish(convert_cloudi_ptr_to_point_cloud2(merged_conveyors, frame, this));
    detection_3d_pub_->publish(*conveyors_candidates_detection_3d_msg);
    auto density_segmentation_start = std::chrono::high_resolution_clock::now();

    auto histogram =
        create_histogram(merged_conveyors_without_intensity, forward_resolution);

    if (not histogram.data.size() or not histogram.data[0].size() ) {
        RCLCPP_WARN(get_logger(), "Cannot create a front histogram.");
        clear_markers(frame);
        return;
    }
    auto clustered_histogram = threshold_histogram(histogram, forward_histogram_min, forward_histogram_max);
    // auto low_density_cloud = filter_with_density_on_x_image(merged_conveyors_without_intensity, clustered_histogram,
    //                                                         forward_resolution, tunnel_width, tunnel_height);
    // auto rotated_for_ground_histogram = rotate(merged_conveyors_without_intensity, 0.0, -M_PI / 2.0, 0.0);
    // auto ground_histogram =
    //     create_histogram(rotated_for_ground_histogram, ground_resolution, tunnel_width, tunnel_length);
    
    // if (not ground_histogram.size() or not ground_histogram[0].size() ) {
    //     RCLCPP_WARN(get_logger(), "Cannot create a top histogram.");
    //     clear_markers(frame);
    //     return;
    // }
    // auto clustered_ground_histogram = segment_local_peeks(ground_histogram, 10, 3);
    // auto high_density_top_cloud = filter_with_density_on_z_image(
    //     merged_conveyors_without_intensity, clustered_ground_histogram, ground_resolution, tunnel_width, tunnel_length);

    // auto merged_density_cloud{
    //     merge_clouds_and_remove_simillar_points({low_density_cloud, high_density_top_cloud}, 0.0001)};
    // auto density_segmentation_end = std::chrono::high_resolution_clock::now();
    // density_segmentation_duration_count =
    //     std::chrono::duration_cast<std::chrono::microseconds>(density_segmentation_end - density_segmentation_start)
    //         .count();
    forward_density_histogram_pub_->publish(create_image_from_histogram(histogram));
    // forward_density_clustered_histogram_pub_->publish(create_image_from_histogram(clustered_histogram));
    // forward_hist_filtered_pub_->publish(convert_cloud_ptr_to_point_cloud2(low_density_cloud, frame, this));
    // ground_density_clustered_histogram_pub_->publish(create_image_from_histogram(clustered_ground_histogram));
    // ground_density_histogram_pub_->publish(create_image_from_histogram(ground_histogram));
    // top_hist_filtered_pub_->publish(convert_cloud_ptr_to_point_cloud2(high_density_top_cloud, frame, this));
    // merged_density_clouds_pub_->publish(convert_cloud_ptr_to_point_cloud2(merged_density_cloud, frame, this));

    // auto clusterization_start = std::chrono::high_resolution_clock::now();
    // CloudIPtrs clustered_clouds = clusteler.euclidean(high_density_top_cloud);
    // max_detected_legs = std::max(max_detected_legs, clustered_clouds.size());
    // CloudIPtr merged_clustered_cloud(new CloudI);

    // std::list<EllipsoidInfo> ellipsoids_infos;
    // std::stringstream points_stream;
    // for (auto& clustered_cloud : clustered_clouds) {
    //     if (clustered_cloud->size()) {
    //         *merged_clustered_cloud += *clustered_cloud;
    //     }
    // }

    // auto clusterization_end = std::chrono::high_resolution_clock::now();
    // clusterization_duration_count =
    //     std::chrono::duration_cast<std::chrono::microseconds>(clusterization_end - clusterization_start).count();
    // // clustered_conveyor_pub_->publish(convert_cloudi_ptr_to_point_cloud2(merged_clustered_cloud, frame, this));

    // auto estimation_start = std::chrono::high_resolution_clock::now();
    // for (auto& clustered_cloud : clustered_clouds) {
    //     if (clustered_cloud->size()) {
    //         auto ellipsoide_info = get_ellipsoid_and_center(clustered_cloud);
    //         ellipsoids_infos.push_back(ellipsoide_info);
    //     }
    // }
    // auto estimation_end = std::chrono::high_resolution_clock::now();
    // estimation_duration_count =
    //     std::chrono::duration_cast<std::chrono::microseconds>(estimation_end - estimation_start).count();

    // auto classification_start = std::chrono::high_resolution_clock::now();
    // auto classificated_ellipsoides_info = classificate(ellipsoids_infos);
    // auto classification_end = std::chrono::high_resolution_clock::now();
    // classification_duration_count =
    //     std::chrono::duration_cast<std::chrono::microseconds>(classification_end - classification_start).count();

    // auto bounding_boxes_msg = make_bounding_boxes_from_pointclouds(clustered_clouds, frame);
    // auto spheres = make_markers_from_ellipsoids_infos(classificated_ellipsoides_info);
    // bounding_box_pub_->publish(*bounding_boxes_msg);
    // markers_pub_->publish(*spheres);
    // only_legs_pub_->publish(convert_cloudi_ptr_to_point_cloud2(merged_clustered_cloud, frame, this));

    // ++counter;
    // save_data_to_yaml(classificated_ellipsoides_info);
    // auto end = std::chrono::high_resolution_clock::now();
    // auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // RCLCPP_DEBUG_STREAM(get_logger(), "Callback Took: " << microseconds / 10e6 << "s");
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

void ObjectDetection::clear_markers(const std::string &frame_name){
    vision_msgs::msg::Detection3DArray empty;
    empty.header.frame_id = frame_name;
    empty.header.stamp = get_clock()->now();
    detection_3d_pub_->publish(empty);
}

Histogram ObjectDetection::create_histogram(CloudPtr cloud, double resolution) {
    Histogram histogram_image;
    auto compare_y = [](const Point lhs, const Point rhs) { return lhs.y < rhs.y; };
    auto compare_z = [](const Point lhs, const Point rhs) { return lhs.z < rhs.z; };

    auto point_with_max_y = *std::max_element(cloud->points.begin(), cloud->points.end(), compare_y);
    auto point_with_max_z = *std::max_element(cloud->points.begin(), cloud->points.end(), compare_z);

    histogram_image.resolution = resolution;
    histogram_image.width = 2 *  point_with_max_y.y;
    histogram_image.height = point_with_max_z.z;
    histogram_image.image_width = std::max(static_cast<std::int64_t>(histogram_image.width / histogram_image.resolution ), (std::int64_t)0);
    histogram_image.image_height = std::max(static_cast<std::int64_t>(histogram_image.height / histogram_image.resolution ), (std::int64_t)0);

    if(histogram_image.image_width == 0 or histogram_image.image_height == 0){
        return histogram_image;
    }

    histogram_image.data.resize(histogram_image.image_height);

    for (auto& column : histogram_image.data) {
        column.resize(histogram_image.image_width);
    }

    for (const auto& point : cloud->points) {
        const auto image_width_pos = static_cast<std::size_t>((histogram_image.width / 2 - point.y) / resolution);
        const auto image_height_pos = static_cast<std::size_t>(point.z / histogram_image.resolution );

        if (image_width_pos >= histogram_image.image_width or image_height_pos >= histogram_image.image_height) {
            continue;
        }

        ++histogram_image.data[image_height_pos][image_width_pos];
    }
    return histogram_image;
}

sensor_msgs::msg::Image ObjectDetection::create_image_from_histogram(const Histogram& histogram) {
    sensor_msgs::msg::Image image_msg;
    image_msg.height = histogram.data.size();
    image_msg.width = histogram.data.begin()->size();
    image_msg.encoding = "mono8";

    if(image_msg.height == 0 or image_msg.width == 0){
        return image_msg;
    }

    image_msg.step = image_msg.width;
    image_msg.data.resize(image_msg.height * image_msg.step, 0);
    // TODO: another function
    auto max_element = std::numeric_limits<std::size_t>::min();
    for (const auto& col : histogram.data) {
        const auto col_max = *std::max_element(col.begin(), col.end());
        max_element = std::max(col_max, max_element);
    }
    if (not max_element) {
        return image_msg;
    }

    for (std::size_t i = 0; i < image_msg.height; ++i) {
        for (std::size_t j = 0; j < histogram.data[i].size(); ++j) {
            uint8_t intensity = static_cast<uint8_t>(255 * histogram.data[i][j] / max_element);
            image_msg.data[(image_msg.height - i - 1) * image_msg.step + j] = intensity;
        }
    }

    return image_msg;
}

void ObjectDetection::save_histogram_to_file(const Histogram& histogram, const std::string& file_name) {
    std::ofstream file("/home/rabin/Documents/obsidian/inz/inżynierka/ws/to_histogram_pcds/densities/densities_" +
                       file_name + ".txt");
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file." << std::endl;
        return;
    }
    for (const auto& col : histogram.data) {
        for (const auto& density : col) {
            if (density > 0) file << density << "\n";
        }
    }
    file.close();
}

void ObjectDetection::save_densities_to_file(const std::vector<std::size_t>& densities, const std::string& path) {
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file." << std::endl;
        return;
    }

    for (const auto& d : densities) {
        file << d << "\n";
    }
    file.close();
}

CloudPtr ObjectDetection::filter_with_density_on_x_image(CloudPtr cloud, const Histogram& histogram) {
    const auto width = histogram.width;
    const auto height = histogram.height;
    const auto image_width = histogram.image_width;
    const auto image_height = histogram.image_height;
    const auto resolution = histogram.resolution;

    auto low_density_cloud = CloudPtr(new Cloud);
    for (const auto& point : cloud->points) {
        const auto image_width_pos = static_cast<std::size_t>((width / 2 + point.y) / resolution);
        const auto image_height_pos = static_cast<std::size_t>(point.z / resolution);

        if (image_width_pos >= image_width or image_height_pos >= image_height) {
            continue;
        }
        if (histogram.data[image_height_pos][image_width_pos]) {
            low_density_cloud->points.push_back(point);
        }
    }
    low_density_cloud->width = low_density_cloud->size();
    low_density_cloud->height = 1;
    return low_density_cloud;
}

CloudPtr ObjectDetection::filter_with_density_on_z_image(CloudPtr cloud, const Histogram& histogram) {
    const auto width = histogram.width;
    const auto height = histogram.height;
    const auto image_width = histogram.image_width;
    const auto image_height = histogram.image_height;
    const auto resolution = histogram.resolution;

    auto low_density_cloud = CloudPtr(new Cloud);
    for (const auto& point : cloud->points) {
        const auto image_width_pos = static_cast<std::size_t>((width / 2 + point.y) / resolution);
        const auto image_lenght_pos = static_cast<std::size_t>(point.x / resolution);

        if (image_width_pos >= image_width or image_lenght_pos >= image_height) {
            continue;
        }
        if (histogram.data[image_lenght_pos][image_width_pos]) {
            low_density_cloud->points.push_back(point);
        }
    }
    low_density_cloud->width = low_density_cloud->size();
    low_density_cloud->height = 1;
    return low_density_cloud;
}

Histogram ObjectDetection::threshold_histogram(const Histogram& histogram, std::size_t min, std::size_t max) {
    auto clustered_histogram(histogram);
    for (auto& col : clustered_histogram.data) {
        for (auto& density : col) {
            auto clamped_density = std::clamp(density, min, max);
            if (density != clamped_density and density != 0) {
                density = 0;
            }
        }
    }
    return clustered_histogram;
}

Histogram ObjectDetection::remove_low_density_columns(const Histogram& histogram, std::size_t threshold) {
    auto clustered_histogram(histogram);

    for (std::size_t i = 0; i < clustered_histogram.data[0].size(); ++i) {
        std::size_t sum = 0;
        for (std::size_t j = 0; j < clustered_histogram.data.size(); ++j) {
            sum += clustered_histogram.data[j][i];
        }
        if (sum < threshold) {
            for (std::size_t j = 0; j < clustered_histogram.data.size(); ++j) {
                clustered_histogram.data[j][i] = 0;
            }
        }
        // RCLCPP_INFO_STREAM(get_logger(), "Column i: " << i << " has sum: " << sum);
    }
    return clustered_histogram;
}

CloudPtr ObjectDetection::merge_clouds_and_remove_simillar_points(CloudPtrs clouds, double eps) {
    CloudPtr new_cloud(new Cloud);
    for (const auto& cloud : clouds) {
        *new_cloud += *cloud;
    }
    for (std::size_t i = 0; i < new_cloud->size(); ++i) {
        for (std::size_t j = 0; j < new_cloud->size(); ++j) {
            if (i == j) continue;
            const double distance = pcl::euclideanDistance(new_cloud->points[i], new_cloud->points[j]);
            if (distance < eps) {
                new_cloud->points.erase(new_cloud->points.begin() + j);
            }
        }
    }
    new_cloud->height = new_cloud->points.size();
    new_cloud->width = 1;
    return new_cloud;
}

BoundingBoxArrayPtr ObjectDetection::make_bounding_boxes_from_pointclouds(const CloudIPtrs& clustered_clouds,
                                                                          const std::string& frame_name) {
    BoundingBoxArrayPtr bounding_boxes(new vision_msgs::msg::BoundingBox3DArray);
    for (const auto& leg : clustered_clouds) {
        vision_msgs::msg::BoundingBox3D bounding_box;
        const float& max_value = std::numeric_limits<float>::max();
        const float& min_value = -std::numeric_limits<float>::max();
        Point max_coords{min_value, min_value, min_value};
        Point min_coords{max_value, max_value, max_value};
        for (const auto& point : leg->points) {
            max_coords.x = std::max(point.x, max_coords.x);
            max_coords.y = std::max(point.y, max_coords.y);
            max_coords.z = std::max(point.z, max_coords.z);

            min_coords.x = std::min(point.x, min_coords.x);
            min_coords.y = std::min(point.y, min_coords.y);
            min_coords.z = std::min(point.z, min_coords.z);
        }

        bounding_box.size.x = std::abs(max_coords.x - min_coords.x);
        bounding_box.size.y = std::abs(max_coords.y - min_coords.y);
        bounding_box.size.z = std::abs(max_coords.z - min_coords.z);

        bounding_box.center.position.x = min_coords.x + bounding_box.size.x / 2.0;
        bounding_box.center.position.y = min_coords.y + bounding_box.size.y / 2.0;
        bounding_box.center.position.z = min_coords.z + bounding_box.size.z / 2.0;

        bounding_boxes->boxes.push_back(bounding_box);
    }
    bounding_boxes->header.frame_id = frame_name;
    bounding_boxes->header.stamp = get_clock()->now();
    return bounding_boxes;
}

vision_msgs::msg::ObjectHypothesisWithPose ObjectDetection::score_conveyor(const vision_msgs::msg::BoundingBox3D bbox) {
    vision_msgs::msg::ObjectHypothesisWithPose object;
    const double conveyor_position_z = 0.38;
    const double conveyor_height = 0.60;
    auto z_error = std::abs(bbox.center.position.z - conveyor_position_z) / conveyor_position_z;
    auto height_error = std::abs(bbox.size.z - conveyor_height) / conveyor_height;
    auto whole_error = z_error + height_error;

    object.hypothesis.score = std::max({0.0, 1.0 - whole_error});
    object.hypothesis.class_id = "conveyor";
    return object;
}

Detection3DArrayPtr ObjectDetection::detect_conveyors(const CloudIPtrs& clustered_clouds,
                                                      const std::string& frame_name) {
    auto bboxes = make_bounding_boxes_from_pointclouds(clustered_clouds, frame_name);
    Detection3DArrayPtr detections(new vision_msgs::msg::Detection3DArray);
    detections->header.frame_id = frame_name;
    detections->header.stamp = get_clock()->now();

    int i = 0;
    for (const auto& bbox : bboxes->boxes) {
        vision_msgs::msg::Detection3D detection;
        detection.header = detections->header;
        detection.bbox = bbox;
        detection.results.push_back(score_conveyor(bbox));
        detections->detections.push_back(detection);
    }

    return detections;
}

CloudPtr ObjectDetection::remove_far_points_from_ros2bag_converter_bug(CloudPtr cloud, double max_distance) {
    auto new_cloud = CloudPtr(new Cloud);
    for (const auto& point : cloud->points) {
        const auto distance = pcl::euclideanDistance(point, Point{0, 0, 0});
        if (distance < max_distance) {
            new_cloud->points.push_back(point);
        }
    }
    new_cloud->height = new_cloud->points.size();
    new_cloud->width = 1;
    return new_cloud;
}

CloudPtr ObjectDetection::remove_right_points(CloudPtr cloud) {
    auto new_cloud = CloudPtr(new Cloud);
    for (const auto& point : cloud->points) {
        if (point.y > 0.0) {
            new_cloud->points.push_back(point);
        }
    }
    new_cloud->height = cloud->points.size();
    new_cloud->width = 1;
    return new_cloud;
}

CloudPtr ObjectDetection::get_points_from_bounding_boxes(CloudPtr cloud, BoundingBoxArrayPtr boxes) {
    CloudPtr new_cloud(new Cloud);
    for (const auto& point : cloud->points) {
        for (const auto& box : boxes->boxes) {
            if (is_point_inside_box(point, box)) {
                new_cloud->points.push_back(point);
            }
        }
    }

    new_cloud->height = new_cloud->points.size();
    new_cloud->width = 1;
    return new_cloud;
}

std::pair<CloudPtr, CloudPtr> ObjectDetection::filter_ground_and_get_normal_and_height(CloudPtr cloud, int sac_model,
                                                                                       int iterations, double radius,
                                                                                       Eigen::Vector3d& normal,
                                                                                       double& ground_height,
                                                                                       double eps) {
    // Segment ground
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    CloudPtr ground(new Cloud);
    CloudPtr without_ground(new Cloud);

    pcl::SACSegmentation<Point> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(sac_model);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(radius);
    seg.setMaxIterations(iterations);
    seg.setEpsAngle(eps);
    pcl::ExtractIndices<Point> extract;
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        // TODO: check it above
        return {nullptr, nullptr};
    }
    extract.setNegative(true);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*without_ground);
    extract.setNegative(false);
    extract.filter(*ground);

    normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
    ground_height = coefficients->values[3];
    return {ground, without_ground};
}

CloudPtr ObjectDetection::align_to_normal(CloudPtr cloud, const Eigen::Vector3d& normal, double ground_height) {
    const auto& up_vector = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d axis = normal.cross(up_vector).normalized();
    float angle = acos(normal.dot(up_vector) / (normal.norm() * up_vector.norm()));

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(angle, axis);
    RCLCPP_DEBUG_STREAM(get_logger(), "Rotation matrix: \n" << rotation_matrix);
    Eigen::Vector3d rpy = rotation_matrix.normalized().eulerAngles(2, 1, 0);
    auto transformed_cloud = translate(rotate(cloud, rpy[0], rpy[1], rpy[2] + 0.06), 0, 0, ground_height);
    transformed_cloud->height = transformed_cloud->points.size();
    transformed_cloud->width = 1;
    return transformed_cloud;
}

ObjectDetection::EllipsoidInfo ObjectDetection::get_ellipsoid_and_center(CloudIPtr cloud) {
    const float& max_value = std::numeric_limits<float>::max();
    const float& min_value = -std::numeric_limits<float>::max();
    Point max_coords{min_value, min_value, min_value};
    Point min_coords{max_value, max_value, max_value};
    for (const auto& point : cloud->points) {
        max_coords.x = std::max(point.x, max_coords.x);
        max_coords.y = std::max(point.y, max_coords.y);
        max_coords.z = std::max(point.z, max_coords.z);

        min_coords.x = std::min(point.x, min_coords.x);
        min_coords.y = std::min(point.y, min_coords.y);
        min_coords.z = std::min(point.z, min_coords.z);
    }
    Ellipsoid ellipsoid;
    ellipsoid.radius_x = (max_coords.x - min_coords.x) / 2;
    ellipsoid.radius_y = (max_coords.y - min_coords.y) / 2;
    ellipsoid.radius_z = (max_coords.z - min_coords.z) / 2;
    Point center;
    center.x = min_coords.x + ellipsoid.radius_x;
    center.y = min_coords.y + ellipsoid.radius_y;
    center.z = min_coords.z + ellipsoid.radius_z;
    RCLCPP_DEBUG_STREAM(get_logger(), "Ellipsoid radiuses: " << ellipsoid << "\n ellipsoid center: \n" << center);

    return {ellipsoid, center, "unknown"};
}

std::list<ObjectDetection::EllipsoidInfo> ObjectDetection::classificate(
    const std::list<ObjectDetection::EllipsoidInfo>& ellipsoids_infos) {
    std::list<ObjectDetection::EllipsoidInfo> classified_ellipsoids_infos{ellipsoids_infos};
    for (auto& info : classified_ellipsoids_infos) {
        const double& model_x_size = 0.2;  // z modelu
        const double& model_y_size = 0.4;
        // 0.05 is a ground radius filter
        const double& model_z_size = 0.6 - 0.05;
        const double& model2_z_size = 0.7 - 0.05;
        RCL_UNUSED(model_x_size);

        auto z_min = info.center.z - info.radiuses.radius_z;
        // From ground +- 10cm (plane segmentation) to full 0.6m size
        // RCLCPP_INFO_STREAM()
        if (2 * info.radiuses.radius_y > 0.1) {
            if ((std::abs(2 * info.radiuses.radius_z - model_z_size) < 0.1 * model_z_size) && info.center.y > 0) {
                info.class_name = "0.6m_height_support";
            }
            // From ground to full 0.7m size
            else if ((std::abs(2 * info.radiuses.radius_z - model2_z_size) < 0.1 * model2_z_size) &&
                     info.center.y < 0) {
                info.class_name = "0.7m_height_support";
            }
        } else {
            info.class_name = "unknown";
        }
    }
    return classified_ellipsoids_infos;
}

void ObjectDetection::save_data_to_yaml(const std::list<EllipsoidInfo>& ellipsoids_infos) {
    YAML::Node frame_node;
    YAML::Node yaml_node;
    std::size_t detected_count;
    frame_node["time"] = rclcpp::Clock{}.now().seconds();
    for (const auto& info : ellipsoids_infos) {
        YAML::Node ellipsoidNode;
        ellipsoidNode["position"]["x"] = info.center.x;
        ellipsoidNode["position"]["y"] = info.center.y;
        ellipsoidNode["position"]["z"] = info.center.z;

        ellipsoidNode["major_axes"]["x"] = info.radiuses.radius_x;
        ellipsoidNode["major_axes"]["y"] = info.radiuses.radius_y;
        ellipsoidNode["major_axes"]["z"] = info.radiuses.radius_z;

        ellipsoidNode["class"] = info.class_name;
        if (info.class_name != "unknown") {
            ++detected_count;
        }

        frame_node["detected_ellipsoids"].push_back(ellipsoidNode);
    }
    std::ofstream file(filename, std::ios::app);

    auto start = std::chrono::high_resolution_clock::now();
    frame_node["durations"]["normalization"] = normalization_duration_count / 10e6;
    frame_node["durations"]["density_segmentation"] = density_segmentation_duration_count / 10e6;
    frame_node["durations"]["clusterization"] = clusterization_duration_count / 10e6;
    frame_node["durations"]["classification"] = classification_duration_count / 10e6;
    frame_node["durations"]["estimation"] = estimation_duration_count / 10e6;
    auto processing_duration = normalization_duration_count + density_segmentation_duration_count +
                               clusterization_duration_count + classification_duration_count +
                               estimation_duration_count;
    frame_node["durations"]["processing"] = processing_duration / 10e6;

    frame_node["filters_point_sizes"]["original"] = original_points_count;
    frame_node["filters_point_sizes"]["5m_filter"] = filter_further_than_5m_points_count;
    frame_node["filters_point_sizes"]["ground_filter"] = filter_ground_points_count;
    frame_node["filters_point_sizes"]["roi"] = roi_points_count;

    yaml_node.push_back(frame_node);
    if (file.is_open()) {
        file << yaml_node << std::endl;
        auto end = std::chrono::high_resolution_clock::now();
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        RCLCPP_DEBUG_STREAM(get_logger(), "Data saved to file. Saving took: " << microseconds / 10e3 << "ms");
    } else {
        RCLCPP_ERROR(get_logger(), "Cannot save data!");
    }
}

MarkersPtr ObjectDetection::make_markers_from_ellipsoids_infos(const std::list<EllipsoidInfo>& ellipsoids_infos) {
    auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();
    std::size_t count_ = 0;

    for (const auto& info : ellipsoids_infos) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "velodyne";
        marker.ns = "spheres";
        marker.id = count_;

        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = info.center.x;
        marker.pose.position.y = info.center.y;
        marker.pose.position.z = info.center.z;

        marker.scale.x = info.radiuses.radius_x * 2;
        marker.scale.y = info.radiuses.radius_y * 2;
        marker.scale.z = info.radiuses.radius_z * 2;  // Height of the cylinder
        marker.color.a = 0.2;                         // Alpha
        marker.color.r = 0.0;                         // Red
        marker.color.g = 0.0;                         // Green
        marker.color.b = 0.0;                         // Blue

        if (info.class_name == "0.6m_height_support") {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            RCLCPP_DEBUG_STREAM(get_logger(), "Found: 0.6m_height_support");
        } else if (info.class_name == "0.7m_height_support") {
            marker.color.r = 0.7;
            marker.color.g = 1.0;
            marker.color.b = 0.3;
            RCLCPP_DEBUG_STREAM(get_logger(), "Found: 0.7m_height_support");

        } else if (info.class_name == "unknown") {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        count_++;
        marker_array_msg->markers.push_back(marker);
    }

    for (std::size_t i = ellipsoids_infos.size(); i < max_detected_legs; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "velodyne";
        marker.ns = "spheres";
        marker.id = i;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array_msg->markers.push_back(marker);
    }
    return marker_array_msg;
}

std::ostream& operator<<(std::ostream& os, const ObjectDetection::Ellipsoid& ellipsoid) {
    os << "Ellipsoid(" << ellipsoid.radius_x << ", " << ellipsoid.radius_y << ", " << ellipsoid.radius_z << ")";
    return os;
}

Histogram ObjectDetection::segment_local_peeks(const Histogram& histogram, std::size_t slope, std::size_t range) {
    Histogram segmented_histogram(histogram);
    for (auto i = range; i < histogram.data.size() - range; ++i) {
        for (auto j = 0u; j < histogram.data[i].size(); ++j) {
            for (auto k = 1u; k <= range; ++k) {
                if (segmented_histogram.data[i][j] != 0 &&
                    (histogram.data[i][j] < histogram[i - k][j] + slope || histogram.data[i][j] < histogram.data[i + k][j] + slope)) {
                    segmented_histogram.data[i][j] = 0;
                }
            }
        }
    }
    return segmented_histogram;
}
