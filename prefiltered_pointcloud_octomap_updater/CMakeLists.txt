cmake_minimum_required(VERSION 3.1.3)
project(prefiltered_pointcloud_octomap_updater)

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
  moveit_ros_perception
  nodelet
  )
moveit_build_options()

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV)

set(PACKAGE_LIBRARIES
  moveit_prefiltered_pointcloud_octomap_updater_core
  )

catkin_package(
  INCLUDE_DIRS
  include
  ${perception_GL_INCLUDE_DIRS}
  LIBRARIES
  ${PACKAGE_LIBRARIES}
  CATKIN_DEPENDS
  image_transport
  moveit_core
  moveit_msgs
  moveit_ros_occupancy_map_monitor
  moveit_ros_planning
  moveit_ros_perception
  nodelet
  object_recognition_msgs
  roscpp
  sensor_msgs
  tf2_geometry_msgs
  DEPENDS
  EIGEN3
)

include_directories(include
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

set(MOVEIT_LIB_NAME prefiltered_pointcloud_octomap_updater)

add_library(${MOVEIT_LIB_NAME}_core src/prefiltered_pointcloud_octomap_updater.cpp)
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES LINK_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS ${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  FILES
  prefiltered_pointcloud_octomap_updater_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION})
