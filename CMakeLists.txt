cmake_minimum_required(VERSION 2.8.3)
project(desperate_housewife)

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED 
  sensor_msgs
  controller_interface
  control_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  urdf
  roscpp
  kdl_parser
  message_generation
  cmake_modules
  roscpp 
  pcl_conversions
  pcl_ros
  tf_conversions 
  pcl_msgs
  message_generation 
  std_msgs
  geometry_msgs 
  sensor_msgs)

find_package(PCL 1.7.2 REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})


add_definitions(-std=c++11 --all-warnings -O3) 

find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  fittedGeometriesSingle.msg
  fittedGeometries.msg
  PointArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
 )

catkin_package(
  DEPENDS orocos_kdl
  CATKIN_DEPENDS
  controller_interface
  control_msgs
  control_toolbox
  realtime_tools
  urdf
  forward_command_controller
  kdl_parser
  message_runtime
  tf_conversions
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})


add_executable(scene_filtering src/scene_filtering.cpp)
add_dependencies(scene_filtering ${PROJECT_NAME}_gencpp)
target_link_libraries(scene_filtering ${catkin_LIBRARIES}  ${PCL_LIBRARIES} )

add_executable(fit_geometries src/fit_geometries.cpp)
add_dependencies(fit_geometries ${PROJECT_NAME}_gencpp)
target_link_libraries(fit_geometries ${catkin_LIBRARIES}  ${PCL_LIBRARIES} )

add_executable(hand_pose_generator src/hand_pose_generator.cpp)
add_dependencies(hand_pose_generator ${PROJECT_NAME}_gencpp)
target_link_libraries(hand_pose_generator ${catkin_LIBRARIES}  ${PCL_LIBRARIES} )




add_library(${PROJECT_NAME} src/potential_field_control.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} kdl_parser ${orocos_kdl_LIBRARIES})

add_executable(test test/test.cpp)
target_link_libraries(test ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(phobic_camera src/main.cpp src/phobic_camera_old.cpp)
# add_dependencies(phobic_camera ${PROJECT_NAME}_gencpp)
# target_link_libraries(phobic_camera   ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES desperate_housewife_controllers.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
