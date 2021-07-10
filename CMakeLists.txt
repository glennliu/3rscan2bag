cmake_minimum_required(VERSION 3.0.2)
project(3rscan2bag)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Set directories here!
set(OpenCV_DIR /home/lch/tools/opencv-3.4.8/build)
set(RIO_LIB /home/lch/catkin_ws2/src/dataset_tool/3RScan/c++/rio_lib)

## Link libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  rosbag
  cv_bridge
  sensor_msgs
  geometry_msgs
  # image_transport
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${RIO_LIB}/src/rio_lib
)

## Build
add_executable(${PROJECT_NAME}_node src/3rscan2bag_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${RIO_LIB}
  ${RIO_LIB}/build/lib/librio_lib.a
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED YES)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_3rscan2bag.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
