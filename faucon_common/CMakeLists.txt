cmake_minimum_required(VERSION 3.8)
project(faucon_common)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Module Python faucon_common (importable par tous les paquets du workspace)
ament_python_install_package(${PROJECT_NAME})

ament_package()
