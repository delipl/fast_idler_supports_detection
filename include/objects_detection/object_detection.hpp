#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/common/pca.h>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/bounding_box3_d_array.hpp>

#include <objects_detection/cluster_extraction.hpp>
#include <objects_detection/down_sampling.hpp>
#include <objects_detection/ground_removal.hpp>
#include <objects_detection/outlier_removal.hpp>
#include <objects_detection/utils.hpp>

using namespace std::chrono_literals;

class ObjectDetection : public rclcpp::Node {
   public:
    ObjectDetection();
        struct Ellipsoid{
        double radius_x{1.0};
        double radius_y{1.0};
        double radius_z{1.0};
    };
    using EllipsoidInfo = std::pair<Ellipsoid, Point>;

   private:


    void declare_parameters();
    void get_parameters();

    std::string pointcloud_topic_name;

    double tunnel_width;
    double tunnel_height;
    double tunnel_length;

    double roll;
    double pitch;
    double yaw;

    double x;
    double y;
    double z;
    double forward_resolution;
    std::size_t forward_histogram_min;
    std::size_t forward_histogram_max;
    std::size_t forward_column_density_threshold;

    double ground_resolution;
    std::size_t ground_histogram_min;
    std::size_t ground_histogram_max;
    double ground_histogram_a;

    std::size_t max_detected_legs = 0;
    std::size_t counter = 0;
    void create_rclcpp_instances();
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<rclcppCloud>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr ground_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr without_ground_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr transformed_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr tunneled_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr outlier_removal_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr forward_hist_filtered_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr top_hist_filtered_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr clustered_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr clustered_conveyor_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr only_legs_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr merged_density_clouds_pub_;
    rclcpp::Publisher<rclcppCloud>::SharedPtr plane_filter_pub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<vision_msgs::msg::BoundingBox3DArray>::SharedPtr bounding_box_pub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr forward_density_histogram_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr forward_density_clustered_histogram_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ground_density_histogram_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ground_density_histogram_multiplied_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ground_density_clustered_histogram_pub_;

    DownSampling down_sampler;
    OutlierRemoval outlier_remover;
    GroundRemoval ground_remover;
    ClusterExtraction clusteler;
    ClusterExtraction clusteler_conveyor;

    CloudPtr remove_points_beyond_tunnel(CloudPtr cloud);
    CloudPtr filter_with_density_on_x_image(CloudPtr cloud, const Histogram &histogram, double resolution, double width,
                                            double height);
    CloudPtr filter_with_density_on_z_image(CloudPtr cloud, const Histogram &histogram, double resolution, double width,
                                            double length);
    CloudPtr remove_far_points_from_ros2bag_converter_bug(CloudPtr cloud, double max_distance);
    CloudPtr merge_clouds(CloudPtrs clouds, double eps);
    std::pair<CloudPtr, CloudPtr> filter_ground_and_get_normal_and_height(CloudPtr cloud, int sac_model, int iterations, double radius, Eigen::Vector3d &normal, double &ground_height, double eps=0);
    CloudPtr remove_right_points(CloudPtr cloud);
    CloudPtr align_to_normal(CloudPtr cloud, const Eigen::Vector3d& normal, double ground_height);

    Point get_center_of_model(CloudIPtr cloud);
    CloudPtr get_points_from_bounding_boxes(CloudPtr cloud, BoundingBoxArrayPtr boxes);
    // CloudPtr plane_filter(CloudPtr cloud);

    Histogram create_histogram(CloudPtr cloud, double resolution,
                                                         double width, double height);
    Histogram remove_low_density_columns(const Histogram &histogram, std::size_t threshold);
    Histogram threshold_histogram(const Histogram &histogram, std::size_t min, std::size_t max);
    Histogram segment_local_peeks(const Histogram &histogram, std::size_t slope, std::size_t range=1);
    Histogram multiply_histogram_by_exp(const Histogram &histogram, double a);

    void save_histogram_to_file(const Histogram &histogram, const std::string & file_name);
    sensor_msgs::msg::Image create_image_from_histogram(const Histogram &histogram);

    MarkersPtr make_markers_from_pointclouds(const CloudIPtrs &clustered_clouds);
    MarkersPtr make_markers_from_ellipsoids_infos(const std::list<EllipsoidInfo> &ellipsoids_infos);
    BoundingBoxArrayPtr make_bounding_boxes_from_pointclouds(const CloudIPtrs &clustered_clouds, const std::string &frame_name);

    void save_densities_to_file(const std::vector<std::size_t> &densities, const std::string &path);

    EllipsoidInfo get_ellipsoid_and_center(CloudIPtr cloud);

};


// Przeładowanie operatora << dla struktury Ellipsoid
std::ostream& operator<<(std::ostream& os, const ObjectDetection::Ellipsoid& ellipsoid) ;
