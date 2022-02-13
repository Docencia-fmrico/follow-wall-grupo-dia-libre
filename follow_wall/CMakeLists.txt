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

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  src
)

add_library(${PROJECT_NAME} SHARED src/follow_wall/follow_wall.cpp)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(follow_wall_node
src/follow_wall_node.cpp)

target_link_libraries(follow_wall_node ${PROJECT_NAME})

install(TARGETS
  follow_wall_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()

ament_add_gtest(test_node test/test_node.cpp)
target_link_libraries(test_node ${PROJECT_NAME})

