cmake_minimum_required(VERSION 3.5)
project(locomotion_mode)

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

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(urdf REQUIRED)
find_package(rover_msgs REQUIRED)

include_directories(include)

set(dependencies
  "std_msgs"
  "std_srvs"
  "geometry_msgs"
  "sensor_msgs"
  "rclcpp"
  "urdf"
  "rover_msgs")

# add library
add_library(${PROJECT_NAME} STATIC src/locomotion_mode.cpp)

# specify include paths and dependencies
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# copy public headers to destination
install(
  DIRECTORY include/
  DESTINATION include
)

# install target at destination
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()