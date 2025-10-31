cmake_minimum_required(VERSION 3.8)
project(faucon_orchestration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(PROJ_DEPS
  rclcpp
  behaviortree_cpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  ament_index_cpp
)

include_directories(include)



add_library(${PROJECT_NAME}_bt_nodes STATIC
  # src/base_bt_node.cpp
  src/check_system_node.cpp
  src/check_drone_node.cpp
  # src/navigation_node.cpp
  # src/detect_event_node.cpp
  # src/dispatch_uav_node.cpp
  # src/ingest_result_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_bt_nodes ${PROJ_DEPS})


add_executable(orchestrator_bt_node
  src/main.cpp
)
target_link_libraries(orchestrator_bt_node
  ${PROJECT_NAME}_bt_nodes
)
ament_target_dependencies(orchestrator_bt_node ${PROJ_DEPS})

# ----- Installation

install(DIRECTORY include/faucon_orchestration/
  DESTINATION include
)


install(TARGETS
  ${PROJECT_NAME}_bt_nodes
  orchestrator_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(${PROJ_DEPS})
ament_export_include_directories(include)

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
