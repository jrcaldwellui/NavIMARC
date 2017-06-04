cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})


# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11")
#-std=c++11

# Build our plugin
add_library(prop_plugin SHARED prop_plugin.cc)
#target_link_libraries(prop_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
target_link_libraries(prop_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${roscpp_LIBRARIES})
