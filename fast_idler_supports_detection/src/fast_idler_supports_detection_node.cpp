#include <fast_idler_supports_detection/fast_idler_supports_detection.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FastIdlerSupportsDetection>());
    rclcpp::shutdown();
    return 0;
}
