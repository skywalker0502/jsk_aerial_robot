cmake_minimum_required(VERSION 2.8.3)
project(mini_quadrotor)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  )

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(process_cloud_point src/process_point_cloud.cpp)
add_dependencies(process_cloud_point ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(process_cloud_point
  ${catkin_LIBRARIES}
  )




