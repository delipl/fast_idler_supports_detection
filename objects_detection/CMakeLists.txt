cmake_minimum_required(VERSION 3.8)
project(objects_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(vision_msgs REQUIRED)

set(CMAKE_BUILD_TYPE Release)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
)

add_library(detection_library SHARED
  src/utils.cpp
  src/cluster_extraction.cpp
)

target_include_directories(
  detection_library
  PUBLIC
  include
)

target_link_libraries(detection_library
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

ament_target_dependencies(detection_library
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
  vision_msgs
)

install(TARGETS
  detection_library
  DESTINATION lib/
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  detection_library
)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
  vision_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(${PROJECT_NAME}_tests
    test/test_tutorial.cpp
    src/object_detection.cpp
  )
  target_include_directories(${PROJECT_NAME}_tests PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_tests
    rclcpp
    sensor_msgs
    pcl_conversions
  )
  target_link_libraries(${PROJECT_NAME}_tests ${PCL_LIBRARIES} detection_library)
endif()

install(DIRECTORY
  test_data
  launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(object_detection
  src/object_detection.cpp
  src/node.cpp
)

add_executable(save_pointcloud
  src/save_pointcloud.cpp
)

ament_target_dependencies(object_detection
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
  vision_msgs
)

ament_target_dependencies(save_pointcloud
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_include_directories(
  object_detection
  PUBLIC
  include
)

target_include_directories(
  save_pointcloud
  PUBLIC
  include
)

target_link_libraries(object_detection ${PCL_LIBRARIES} detection_library)
target_link_libraries(save_pointcloud ${PCL_LIBRARIES} detection_library)

install(TARGETS
  object_detection
  save_pointcloud
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/clasification_analizer.py
  scripts/histogram.py
  scripts/plot_on_surface.py
  scripts/isometic.py
  scripts/point_cloud_cli.py
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
