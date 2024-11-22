# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(drone)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
	message_generation
)

add_message_files(FILES Pose.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
	geometry_msgs
	message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(flightSystem src/flightSystem.cpp)
target_link_libraries(flightSystem ${catkin_LIBRARIES})

configure_file(scripts/configuration.in.argos scripts/configuration.argos)
