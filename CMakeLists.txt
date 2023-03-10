cmake_minimum_required(VERSION 2.8.3)
project(ud_path_follower)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geographic_visualization_msgs
  geometry_msgs
  geographic_msgs
  nav_msgs
  project11
  project11_navigation
  pluginlib
  roscpp
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ud_path_follower_nav_plugin
  CATKIN_DEPENDS geometry_msgs roscpp project11 project11_navigation  geographic_visualization_msgs 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ud_path_follower_nav_plugin src/ud_path_follower.cpp src/ud_path_follower_nav_plugin.cpp)
add_dependencies(ud_path_follower_nav_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ud_path_follower_nav_plugin ${catkin_LIBRARIES})


install(TARGETS  ud_path_follower_nav_plugin
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
PATTERN ".svn" EXCLUDE
)

install(FILES ud_path_follower_nav_plugin.xml
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

