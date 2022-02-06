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

include_directories(include)


add_library(${PROJECT_NAME} SHARED
follow_wall/src/follow_wall_node.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(nodo_follow src/nodo_follow.cpp)
ament_target_dependencies(nodo_follow ${dependencies})

target_link_libraries(nodo_follow ${PROJECT_NAME})


install(TARGETS
  ${PROJECT_NAME}
  nodo_follow
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY  launch
  DESTINATION share/${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  src
)

ament_export_dependencies(${dependencies})
ament_package()

