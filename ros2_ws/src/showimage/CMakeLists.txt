cmake_minimum_required(VERSION 3.5)

project(showimage)

# Setup Conan
include(${CMAKE_CURRENT_SOURCE_DIR}/../.build/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable( showimage
  src/showimage.cpp)

ament_target_dependencies(showimage
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "rclcpp_components"
  "OpenCV")

target_link_libraries( ${PROJECT_NAME} 
  CONAN_PKG::spdlog
  CONAN_PKG::boost
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
