cmake_minimum_required(VERSION 2.8.12)
project(keyence_experimental)
## add_compile_options(-std=c++11 -Wall -Wextra)
add_compile_options(-std=c++14 -Wall -Wextra)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  pcl_ros
  roscpp
  std_msgs
  moveit_visual_tools
  rviz_visual_tools
)

## Driver makes use of the libsocket libraries
find_package(libsocket REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  ChangeProgram.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
  keyence_library/include
  LIBRARIES
  keyence_impl
  CATKIN_DEPENDS
  message_generation
  message_runtime
  pcl_ros
  std_msgs
  DEPENDS
  libsocket
)

###########
## Build ##
###########

include_directories(
  keyence_library/include
  ${libsocket_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  after_1st_scanner
  src/after_1st_scanner.cpp
)
add_dependencies(
  after_1st_scanner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_executable(
  concave_scanner
  src/concave_scanner.cpp
)
add_dependencies(
  concave_scanner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_executable(
  convex_scanner
  src/convex_scanner.cpp
)
add_dependencies(
  convex_scanner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_executable(
  keyence_driver_node
  src/keyence_driver_node.cpp
)
add_dependencies(
  keyence_driver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  after_1st_scanner
  keyence_impl
  ${catkin_LIBRARIES}
)
target_link_libraries(
  concave_scanner
  keyence_impl
  ${catkin_LIBRARIES}
)
target_link_libraries(
  convex_scanner
  keyence_impl
  ${catkin_LIBRARIES}
)
target_link_libraries(
  keyence_driver_node
  keyence_impl
  ${catkin_LIBRARIES}
)

add_subdirectory(keyence_library)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
  TARGETS
  after_1st_scanner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS
  concave_scanner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS
  convex_scanner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS
  keyence_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY
  keyence_library/include/
  DESTINATION
  ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY
  config
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  DIRECTORY 
  launch 
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
