cmake_minimum_required(VERSION 2.8.3)
project(chris_create_model)

# Find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

# Added for ROS kinetic
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=gnu++11)
endif()

find_package(catkin REQUIRED COMPONENTS chris_create_node
                                        gazebo_ros
                                        geometry_msgs
                                        nav_msgs
                                        roscpp
                                        sensor_msgs)


catkin_package(INCLUDE_DIRS include
               LIBRARIES chris_create_model
               CATKIN_DEPENDS chris_create_node
                              gazebo_ros
                              geometry_msgs
                              nav_msgs
                              roscpp
                              sensor_msgs
                              urdf
                              xacro)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS})

add_library(chris_create_model src/gazebo_ros_create.cpp)

add_dependencies(chris_create_model chris_create_node_generate_messages_cpp
                                    geometry_msgs_generate_messages_cpp
                                    nav_msgs_generate_messages_cpp
                                    sensor_msgs_generate_messages_cpp)

target_link_libraries(chris_create_model
                      ${catkin_LIBRARIES}
                      ${GAZEBO_LIBRARIES})

install(TARGETS chris_create_model
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY robots
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
