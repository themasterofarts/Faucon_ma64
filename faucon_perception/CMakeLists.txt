cmake_minimum_required(VERSION 3.16)
project(faucon_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)  
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(PROJ_DEPS
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  PCL
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
  OpenCV
)

include_directories(
  include
)


install(
  DIRECTORY include
  DESTINATION include
)

add_library(row_pipeline
  src/pcl/row_pipeline.cpp
  src/pcl/hough_method.cpp
  src/pcl/model_detect.cpp
)

target_include_directories(row_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


target_link_libraries(row_pipeline
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(lidar_calibrator src/lidat_calibrate.cpp)
ament_target_dependencies(lidar_calibrator ${PROJ_DEPS})

add_executable(crop_row_detector src/cluster_rows.cpp)
ament_target_dependencies(crop_row_detector ${PROJ_DEPS})

add_executable(crop_row src/row_clusterer_node.cpp)
ament_target_dependencies(crop_row ${PROJ_DEPS})
target_link_libraries(crop_row
  row_pipeline
  ${PCL_LIBRARIES}
)

add_executable(detect_point src/detect_point_node.cpp)
ament_target_dependencies(detect_point ${PROJ_DEPS})
target_link_libraries(detect_point
row_pipeline
  ${PCL_LIBRARIES}
)

target_link_libraries(lidar_calibrator
  ${PCL_LIBRARIES}
)
install(TARGETS lidar_calibrator
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(crop_row_detector
  ${PCL_LIBRARIES}
)
install(TARGETS crop_row_detector
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(crop_row
  ${PCL_LIBRARIES}
)
install(TARGETS crop_row
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(detect_point
  ${PCL_LIBRARIES}
)
install(TARGETS detect_point
  DESTINATION lib/${PROJECT_NAME}
)

## Algo RANSAC node
add_executable(ransac_node src/ransac_node.cpp src/algo/ransac_method.cpp)
ament_target_dependencies(ransac_node ${PROJ_DEPS})
target_link_libraries(ransac_node
  ${PCL_LIBRARIES}
)
install(TARGETS ransac_node
  DESTINATION lib/${PROJECT_NAME}
)

##

## Algo cluster node
add_executable(cluster_node src/cluster_method_node.cpp src/algo/cluster_method.cpp)
ament_target_dependencies(cluster_node ${PROJ_DEPS})
target_link_libraries(cluster_node
  ${PCL_LIBRARIES}
)
install(TARGETS cluster_node
  DESTINATION lib/${PROJECT_NAME}
)

##


## setup pour les fichiers python

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/mask.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/ExG_ROS.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
