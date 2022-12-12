cmake_minimum_required(VERSION 2.8.3)
project(trajectory_follower)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  roslaunch
  std_msgs
  std_srvs
  uav_abstraction_layer
  nav_msgs
  trajectory_msgs
  handy_tools
  tf2
)
find_package(PythonLibs 2.7)
find_package(Eigen3 REQUIRED)



catkin_package(
 INCLUDE_DIRS
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy tf std_msgs std_srvs uav_abstraction_layer
)

#roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}

)




add_executable(trajectory_follower_node src/trajectory_follower.cpp)



target_link_libraries(trajectory_follower_node
${catkin_LIBRARIES} ${EXTRALIB_BIN} ${PYTHON_LIBRARIES} ${Eigen3_LIBRARIES})

add_dependencies(trajectory_follower_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
