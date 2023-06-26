cmake_minimum_required(VERSION 2.8.3)
project(mini_quadrotor)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  livox_ros_driver2
  )

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(process_pointcloud src/process_pointcloud.cpp)
add_dependencies(process_pointcloud ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(process_pointcloud
  ${catkin_LIBRARIES}
  )




