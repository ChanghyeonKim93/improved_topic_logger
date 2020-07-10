cmake_minimum_required(VERSION 2.8.3)
project(improved_topic_logger)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge

  std_msgs
  message_generation
  
  sensor_msgs
  
  pcl_conversions
  pcl_ros
  message_filters
  image_transport
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  imu_serial.msg
)

## Generate added messages and services with any dependencies listed here.
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
	DEPENDS Eigen
  CATKIN_DEPENDS message_runtime roscpp
)



set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

set(SRC ${SRC} src/main.cpp)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

add_executable (improved_topic_logger src/main.cpp)
target_link_libraries (improved_topic_logger ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries (improved_topic_logger ${Eigen_LIBRARIES} )