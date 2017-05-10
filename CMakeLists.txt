cmake_minimum_required(VERSION 2.8.3)
project(m32_ros)

set(CMAKE_CXX_FLAGS "-D_REENTRANT -std=c++0x -m64 ${CMAKE_CXX_FLAGS}")

find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	tf2_ros
	tf
	cv_bridge
  	nodelet
  	std_msgs
	image_transport
	sensor_msgs
  	eigen_conversions
)

find_package(OpenCV)


catkin_package(
  INCLUDE_DIRS include
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mavros
  CATKIN_DEPENDS roscpp geometry_msgs eigen_conversions tf2_ros
  DEPENDS Boost
)


###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
  src
)

 link_directories(
   ${CMAKE_SOURCE_DIR}/m32_ros/lib   
  )

add_library(inu src/Inuitive.cpp)
target_link_libraries(inu
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  libCommonUtilities.so
  libInuStreams.so
 )
add_executable(m32_ros  src/node.cpp )

target_link_libraries(m32_ros 
  inu
)

