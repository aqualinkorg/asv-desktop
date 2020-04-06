SET(execName zed_test)
CMAKE_MINIMUM_REQUIRED(VERSION 3.5 )
PROJECT(${execName})

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)



option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(ZED 3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

FILE(GLOB_RECURSE SRC_FILES src/*.cpp)

ADD_EXECUTABLE(${execName} ${SRC_FILES})
add_definitions(-std=c++14 -g -O3)

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

TARGET_LINK_LIBRARIES(${execName} ${ZED_LIBS} ${OpenCV_LIBRARIES})

ament_target_dependencies( ${PROJECT_NAME} 
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "std_msgs"
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()