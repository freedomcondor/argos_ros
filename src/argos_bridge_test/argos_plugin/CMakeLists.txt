project(argos_drone_test)
cmake_minimum_required(VERSION 3.16)

# include catkin
include_directories(${catkin_INCLUDE_DIRS})

# find argos
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_BINARY_DIR})

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ARGoSBuildOptions.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ARGoSBuildFlags.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ARGoSBuildChecks.cmake)

# compile test controller
add_library(drone_ros_bridge_controller MODULE
  drone_ros_bridge_controller.h
  drone_ros_bridge_controller.cpp
  # add the debug actuator
  extensions/debug/debug_entity.h
  extensions/debug/debug_entity.cpp
  extensions/debug/debug_default_actuator.h
  extensions/debug/debug_default_actuator.cpp
)
target_link_libraries(drone_ros_bridge_controller
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_drone
    /opt/ros/noetic/lib/libroscpp.so
)

# compile with double # TODO : check necessary later
if(ARGOS_USE_DOUBLE)
  target_compile_definitions(drone_ros_bridge_controller PUBLIC BT_USE_DOUBLE_PRECISION)
endif(ARGOS_USE_DOUBLE)

#-------------------------------------------------------------------
add_library(my_extensions SHARED
  # add the debug actuator
  extensions/debug/debug_entity.h
  extensions/debug/debug_entity.cpp
  extensions/debug/debug_default_actuator.h
  extensions/debug/debug_default_actuator.cpp
  # add the extended pipuck model
  extensions/pipuck_ext/pipuck_ext_entity.h
  extensions/pipuck_ext/pipuck_ext_entity.cpp
  extensions/pipuck_ext/dynamics3d_pipuck_ext_model.h
  extensions/pipuck_ext/dynamics3d_pipuck_ext_model.cpp
  # include the loop function
  extensions/my_loop_functions.h
  extensions/my_loop_functions.cpp
)
target_link_libraries(my_extensions
  argos3core_${ARGOS_BUILD_FOR}
)
if(ARGOS_USE_DOUBLE)
  target_compile_definitions(my_extensions PUBLIC BT_USE_DOUBLE_PRECISION)
endif(ARGOS_USE_DOUBLE)

if(ARGOS_QTOPENGL_FOUND)
  add_library(my_qtopengl_extensions SHARED
    qtopengl_extensions/my_qtopengl_user_functions.h
    qtopengl_extensions/my_qtopengl_user_functions.cpp
  )
endif(ARGOS_QTOPENGL_FOUND)