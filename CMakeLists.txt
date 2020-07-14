cmake_minimum_required(VERSION 2.8.3)
project(active-perception-experiments)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  jason_ros
  jason_ros_msgs
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

add_library(spawn_buoy src/gazebo_plugins/spawn_buoy.cpp)
target_link_libraries(spawn_buoy ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(lifebuoy src/gazebo_plugins/lifebuoy.cpp)
target_link_libraries(lifebuoy ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(camera_switch src/gazebo_plugins/camera_switch.cpp)
target_link_libraries(camera_switch ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(victim src/gazebo_plugins/victim.cpp)
target_link_libraries(victim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
