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
)

include_directories(
  include
)


install(
  DIRECTORY include
  DESTINATION include
)
add_executable(lidar_calibrator src/lidat_calibrate.cpp)
ament_target_dependencies(lidar_calibrator ${PROJ_DEPS})

add_executable(crop_row_detector src/cluster_rows.cpp)
ament_target_dependencies(crop_row_detector ${PROJ_DEPS})

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
