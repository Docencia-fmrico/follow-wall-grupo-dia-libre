cmake_minimum_required(VERSION 3.5)
project(follow_wall)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


set(dependencies
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    geometry_msgs
)

add_executable(follow_wall_node src/follow_wall_node.cpp)
ament_target_dependencies(follow_wall_node ${dependencies})


install(TARGETS
  follow_wall_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_package()

