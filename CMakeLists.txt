cmake_minimum_required(VERSION 3.5)
project(locomotion_mode)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Build Library
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
file(GLOB MY_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
add_library(${PROJECT_NAME} ${MY_SOURCES})

# specify include paths and dependencies
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_include_directories(${PROJECT_NAME} PUBLIC include)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# copy public headers to destination
install(
  DIRECTORY include/
  DESTINATION include
)

# install targets at destination
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

# Add additional Locomotion Modes
# Add executables
add_executable(stop_mode_node src/stop_mode.cpp)
target_link_libraries(stop_mode_node ${PROJECT_NAME})

# install nodes
install(TARGETS
  stop_mode_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()