cmake_minimum_required(VERSION 3.0.2)
project(lxd_berthing_angle)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package(PCL REQUIRED COMPONENT common io filters visualization segmentation)
find_package(Boost REQUIRED)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS PCL
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

include_directories("/usr/include/eigen3/")
link_directories(${PCL_LIBRARY_DIRS})

add_executable(lxd_berthing_angle src/lxd_berthing_angle.cpp src/Cutting_point_cloud.h src/Berthing_angle_calculation.h src/Outlier_noise_filtering.h)
target_link_libraries(lxd_berthing_angle ${catkin_LIBRARIES} ${PCL_LIBRARIES})


