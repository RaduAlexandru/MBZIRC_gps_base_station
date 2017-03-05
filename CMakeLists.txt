cmake_minimum_required(VERSION 2.8)
project(gps_base_station)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(PROJECT_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)		# Header folder
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DAPC_CAPTURE_DEBUG=1")


find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	nav_msgs
	std_srvs
	dji_sdk
	message_filters
	tf
)

add_definitions(-std=gnu++11 -fext-numeric-literals)
find_package(Boost REQUIRED COMPONENTS system)


catkin_package()

include_directories(
	${PROJECT_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
)


add_executable(base_station src/base_station.cpp src/wgs_conversion.cpp)
target_link_libraries(base_station ${catkin_LIBRARIES})
