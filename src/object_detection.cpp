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
    declare_parameter("general.tunnel.width", "4.00");
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
    merged_density_clouds_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/merged_density_clouds", 10);
    plane_filter_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/plane_filter", 10);

    only_legs_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("inz/only_legs", 10);
    bounding_box_pub_ = create_publisher<vision_msgs::msg::BoundingBox3DArray>("inz/bounding_boxes", 10);


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
    auto cloud_raw{convert_point_cloud2_to_cloud_ptr(msg)};
    auto cloud = remove_far_points_from_ros2bag_converter_bug(cloud_raw, 5.0);
    // auto without_right = remove_right_points(cloud);
    // auto without_ground = run_ransac(without_right, pcl::SACMODEL_PLANE, 800, 0.05, true);
    // tunneled_pub_->publish(convert_cloud_ptr_to_point_cloud2(without_ground, frame, this));
    // auto line = run_ransac(without_ground, pcl::SACMODEL_PLANE, 800, 0.05, false);
    // mid360_rotated_pub_->publish(convert_cloud_ptr_to_point_cloud2(line, frame, this));

    auto transformed_cloud = translate(rotate(cloud, roll, pitch, yaw), x, y, z);

    mid360_rotated_pub_->publish(convert_cloud_ptr_to_point_cloud2(transformed_cloud, frame, this));
    auto tunneled_cloud = remove_points_beyond_tunnel(transformed_cloud);
    tunneled_pub_->publish(convert_cloud_ptr_to_point_cloud2(tunneled_cloud, frame, this));

    auto histogram = create_histogram(tunneled_cloud, forward_resolution, tunnel_width, tunnel_height);
    forward_density_histogram_pub_->publish(create_image_from_histogram(histogram));

    auto clustered_histogram = threshold_histogram(histogram, forward_histogram_min, forward_histogram_max);
    auto removed_columns = remove_low_density_columns(clustered_histogram, forward_column_density_threshold);
    forward_density_clustered_histogram_pub_->publish(create_image_from_histogram(removed_columns));

    auto low_density_cloud = filter_with_density_on_x_image(tunneled_cloud, removed_columns, forward_resolution,
                                                            tunnel_width, tunnel_height);

    forward_filtered->publish(convert_cloud_ptr_to_point_cloud2(low_density_cloud, frame, this));

    auto rotated_for_ground_histogram = rotate(tunneled_cloud, 0.0, -M_PI / 2.0, 0.0);
    auto ground_histogram =
        create_histogram(rotated_for_ground_histogram, ground_resolution, tunnel_width, tunnel_length);
    // auto densities = save_histogram_to_file(histogram);
    ground_density_histogram_pub_->publish(create_image_from_histogram(ground_histogram));
    auto multiplied_ground_histogram = multiply_histogram_by_exp(ground_histogram, ground_histogram_a);
    ground_density_histogram_multiplied_pub_->publish(create_image_from_histogram(multiplied_ground_histogram));

    auto clustered_ground_histogram =
        threshold_histogram(multiplied_ground_histogram, ground_histogram_min, ground_histogram_max);
    ground_density_clustered_histogram_pub_->publish(create_image_from_histogram(clustered_ground_histogram));
    auto high_density_top_cloud = filter_with_density_on_z_image(tunneled_cloud, clustered_ground_histogram,
                                                                 ground_resolution, tunnel_width, tunnel_length);

    ground_filtered->publish(convert_cloud_ptr_to_point_cloud2(high_density_top_cloud, frame, this));

    auto merged_dencity_cloud{merge_clouds({low_density_cloud, high_density_top_cloud}, 0.0001)};
    merged_density_clouds_pub_->publish(convert_cloud_ptr_to_point_cloud2(merged_dencity_cloud, frame, this));

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

    auto bounding_boxes_msg = make_bounding_boxes_from_pointclouds(clustered_clouds, frame);
    bounding_box_pub_->publish(*bounding_boxes_msg);

    auto points_inside_boxes = get_points_from_bounding_boxes(tunneled_cloud, bounding_boxes_msg);


    CloudIPtrs clustered_legs = clusteler.euclidean(points_inside_boxes);
    CloudIPtr merged_clustered_legs(new CloudI);
    for (auto& clustered_leg : clustered_legs) {
        if (clustered_leg->size()) {
            *merged_clustered_legs += *clustered_leg;
        }
    }
    only_legs_pub_->publish(convert_cloudi_ptr_to_point_cloud2(merged_clustered_legs, frame, this));

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

void ObjectDetection::save_histogram_to_file(const Histogram& histogram, const std::string& file_name) {
    std::ofstream file(
        "/home/rabin/Documents/obsidian/inz/inżynierka/ws/to_histogram_pcds/densities            /densities_" +
        file_name + ".txt");
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

CloudPtr ObjectDetection::filter_with_density_on_x_image(CloudPtr cloud, const Histogram& histogram, double resolution,
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

CloudPtr ObjectDetection::filter_with_density_on_z_image(CloudPtr cloud, const Histogram& histogram, double resolution,
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

Histogram ObjectDetection::remove_low_density_columns(const Histogram& histogram, std::size_t threshold) {
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
    const double &model_z_size = 0.3;
    float max_z = -std::numeric_limits<float>::max();
    std::size_t count_ = 0;
    for (const auto& leg : clustered_clouds) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "velodyne";
        marker.ns = "cylinders";
        marker.id = count_;
        // marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.mesh_resource = "package://objects_detection/test_data/leg.dae";
        const auto center = get_center_of_model(leg);
        marker.pose.position.x = center.x;
        marker.pose.position.y = center.y;
        marker.pose.position.z = center.z;
        // Zorientować w zależności po której stronie jest
        marker.pose.orientation.w = center.y > 0? 1 : 0;
        marker.pose.orientation.z = center.y > 0? 0 : 1;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;  // Height of the cylinder
        marker.color.a = 1.0;  // Alpha
        marker.color.r = 0.0;  // Red
        marker.color.g = 0.0;  // Green
        marker.color.b = 1.0;  // Blue
        marker.lifetime = rclcpp::Duration(0, 2000);
        count_++;
        marker_array_msg->markers.push_back(marker);
    }
    for (std::size_t i = clustered_clouds.size(); i < max_detected_legs; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "velodyne";
        marker.ns = "cylinders";
        marker.id = i;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array_msg->markers.push_back(marker);
    }
    return marker_array_msg;
}

CloudPtr ObjectDetection::merge_clouds(CloudPtrs clouds, double eps){
    CloudPtr new_cloud(new Cloud);
    for (const auto &cloud : clouds){
        *new_cloud += *cloud;
    }
    for(std::size_t i = 0; i < new_cloud->size(); ++i ){
        for(std::size_t j = 0; j < new_cloud->size(); ++j ){
            if(i == j) continue;
            const double distance = pcl::euclideanDistance(new_cloud->points[i], new_cloud->points[j]);
            if(distance < eps){
                new_cloud->points.erase(new_cloud->points.begin()+j);
            }
        }
    }
    new_cloud->height = new_cloud->points.size();
    new_cloud->width = 1;
    return new_cloud;
}


BoundingBoxArrayPtr ObjectDetection::make_bounding_boxes_from_pointclouds(const CloudIPtrs &clustered_clouds, const std::string &frame_name){
    BoundingBoxArrayPtr bounding_boxes(new vision_msgs::msg::BoundingBox3DArray);
    const double &model_x_size = 0.1; // z modelu
    const double &model_y_size = 0.4;
    const double &model_z_size = 0.3;
    const double &ground_level = 0.0; // po znalezieniu ziemi;

    for (const auto& leg : clustered_clouds) {
        vision_msgs::msg::BoundingBox3D bounding_box;
        const float &max_value = std::numeric_limits<float>::max();
        const float &min_value = -std::numeric_limits<float>::max();
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

        const auto center = get_center_of_model(leg);

        bounding_box.size.x = model_x_size + 0.05; // get more points
        bounding_box.size.y = model_y_size;
        bounding_box.size.z = model_z_size + 0.05; //get higher poinst

        bounding_box.center.position.x = center.x;
        bounding_box.center.position.y = center.y;
        bounding_box.center.position.z = center.z+ 0.05 ; //get higher poinst

        bounding_boxes->boxes.push_back(bounding_box);
    }
    bounding_boxes->header.frame_id = frame_name;
    bounding_boxes->header.stamp = get_clock()->now();
    return bounding_boxes;
}


Histogram ObjectDetection::multiply_histogram_by_exp(const Histogram& histogram, double a) {
    Histogram multipied_histogram(histogram);
    for (auto i = 0u; i < multipied_histogram.size(); ++i) {
        for (auto j = 0u; j < multipied_histogram[i].size(); ++j) {
            multipied_histogram[i][j] *= std::exp(a * i / multipied_histogram.size());
        }
    }
    return multipied_histogram;
}

CloudPtr ObjectDetection::remove_far_points_from_ros2bag_converter_bug(CloudPtr cloud, double max_distance){
    auto new_cloud = CloudPtr(new Cloud);
    for (const auto& point : cloud->points) {
        const auto distance = pcl::euclideanDistance(point, Point{0, 0, 0});
        if (distance < max_distance) {
            new_cloud->points.push_back(point);
        }
    }
    new_cloud->height = cloud->points.size();
    new_cloud->width = 1;
    return new_cloud;
}

CloudPtr ObjectDetection::remove_right_points(CloudPtr cloud){
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

Point ObjectDetection::get_center_of_model(CloudIPtr cloud){
    Point center{0.0, 0.0, 0.0};
    const double &model_height = 0.3;
    const double &model_y_highest_point = 0.08; 
    Point highest_point{0, 0, -std::numeric_limits<float>::max()};
    for (const auto& point : cloud->points) {
        center.x += point.x;
        center.y += point.y;
        if(highest_point.z < point.z){
            highest_point.x = point.x;
            highest_point.y = point.y;
            highest_point.z = point.z;
        }
    }
    const auto& size = cloud->points.size();
    center.x /= size;
    center.y = model_y_highest_point;
    center.y *= highest_point.y > 0 ? 1 : -1;
    center.y += highest_point.y;
    center.z = highest_point.z - model_height/2;
    return center;
}



CloudPtr ObjectDetection::get_points_from_bounding_boxes(CloudPtr cloud, BoundingBoxArrayPtr boxes){
    CloudPtr new_cloud(new Cloud);
    for(const auto &point : cloud->points){
        for(const auto &box: boxes->boxes){
            if(is_point_inside_box(point, box)){
                new_cloud->points.push_back(point);
            }
        }
    }

    new_cloud->height = new_cloud->points.size();
    new_cloud->width = 1;
    return new_cloud;
}

CloudPtr ObjectDetection::run_ransac(CloudPtr cloud, int sac_model, int iterations, double radius, bool negative, double eps){
    // Segment ground
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    CloudPtr new_cloud(new Cloud);
    pcl::SACSegmentation<Point> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (sac_model);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (radius);
    seg.setMaxIterations(iterations);
    seg.setEpsAngle(eps);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        // break;
    }
    extract.setNegative (negative);
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.filter(*new_cloud);
    return new_cloud;
}
