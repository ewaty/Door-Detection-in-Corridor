cmake_minimum_required(VERSION 2.8.3)
project(praca_magisterska)

find_package(orocos_kdl REQUIRED)
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenNI_INCLUDEDIR}
  ${orocos_kdl_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(pointcloud_rgb pointcloud_rgb.cpp)
target_link_libraries(pointcloud_rgb ${catkin_LIBRARIES})

add_executable(people_detection people_detection.cpp)
add_dependencies(people_detection geometry_msgs_gencpp)
target_link_libraries(people_detection ${catkin_LIBRARIES})

install(TARGETS people_detection RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES people_detection.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

