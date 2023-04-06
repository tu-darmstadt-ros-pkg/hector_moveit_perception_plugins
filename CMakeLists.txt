cmake_minimum_required(VERSION 3.1.3)
project(hector_moveit_perception_plugins)

option(WITH_OPENGL "Build the parts that depend on OpenGL" ON)

find_package(Boost REQUIRED thread)

if(WITH_OPENGL)
  # Prefer newer vendor-specific OpenGL library
  if (POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
  endif()
  find_package(OpenGL REQUIRED)
  find_package(GLEW REQUIRED)
  find_package(GLUT REQUIRED)
  set(gl_LIBS ${gl_LIBS} ${OPENGL_LIBRARIES})
  set(SYSTEM_GL_INCLUDE_DIRS ${GLEW_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
endif(WITH_OPENGL)

if(APPLE)
  find_package(X11 REQUIRED)
endif(APPLE)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  moveit_core
  roscpp
  rosconsole
  urdf
  message_filters
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  pluginlib
  image_transport
  object_recognition_msgs
  sensor_msgs
  moveit_msgs
  moveit_ros_occupancy_map_monitor
  moveit_ros_planning
  nodelet
)
moveit_build_options()

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV)

set(PACKAGE_LIBRARIES
  moveit_pointcloud_octomap_updater_core
)

catkin_package(
  INCLUDE_DIRS
    pointcloud_octomap_updater/include
    ${perception_GL_INCLUDE_DIRS}
  LIBRARIES
    ${PACKAGE_LIBRARIES}
  CATKIN_DEPENDS
    image_transport
    moveit_core
    moveit_msgs
    moveit_ros_occupancy_map_monitor
    moveit_ros_planning
    nodelet
    object_recognition_msgs
    roscpp
    sensor_msgs
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(pointcloud_octomap_updater/include
                    ${perception_GL_INCLUDE_DIRS}
                    )
include_directories(SYSTEM
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${SYSTEM_GL_INCLUDE_DIR}
                    ${X11_INCLUDE_DIR}
                    )

add_subdirectory(pointcloud_octomap_updater)

install(
  FILES
    pointcloud_octomap_updater_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})
