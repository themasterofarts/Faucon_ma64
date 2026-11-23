cmake_minimum_required(VERSION 3.8)
project(faucon_ihm)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY ihm/dist/
  DESTINATION share/${PROJECT_NAME}/ihm
  OPTIONAL   
)

install(
  DIRECTORY ros2_ihm/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_EXECUTE GROUP_READ
)

install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
