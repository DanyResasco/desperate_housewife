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
  lwr_controllers
  calibration
)

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

# add a target to generate API documentation with Doxygen


find_package(Doxygen)
if(DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.in @ONLY)
  add_custom_target(doc
  ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/doxygen.in
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif(DOXYGEN_FOUND)


add_message_files(
  FILES
  fittedGeometriesSingle.msg
  fittedGeometriesArray.msg
  handPoseSingle.msg
  handPosesArray.msg
  Error_msg.msg
)

add_service_files(
  FILES
goPose.srv
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
  lwr_hw
  lwr_controllers
  calibration
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})


add_executable(scene_filtering src/scene_filtering.cpp)
add_dependencies(scene_filtering ${PROJECT_NAME}_gencpp)
target_link_libraries(scene_filtering ${catkin_LIBRARIES}  ${PCL_LIBRARIES} )

add_executable(fit_geometries src/fit_geometries_node.cpp src/fit_geometries.cpp)
add_dependencies(fit_geometries ${PROJECT_NAME}_gencpp)
target_link_libraries(fit_geometries ${catkin_LIBRARIES}  ${PCL_LIBRARIES} )


add_library(${PROJECT_NAME} src/potential_field_control.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} kdl_parser ${orocos_kdl_LIBRARIES})

add_executable(ball_test src/ball_main.cpp src/ball_node.cpp)
add_dependencies(ball_test ball_test_gencpp ${PROJECT_NAME})
target_link_libraries(ball_test ${PROJECT_NAME} ${catkin_LIBRARIES} kdl_parser ${orocos_kdl_LIBRARIES})

add_executable(grid src/grid_main.cpp src/grid_node.cpp)
add_dependencies(grid grid_gencpp ${PROJECT_NAME})
target_link_libraries(grid ${PROJECT_NAME} ${catkin_LIBRARIES} kdl_parser ${orocos_kdl_LIBRARIES})

add_executable(test_send_obst src/test_send_obst.cpp )
add_dependencies(test_send_obst ${PROJECT_NAME}_gencpp)
target_link_libraries(test_send_obst ${catkin_LIBRARIES} kdl_parser ${orocos_kdl_LIBRARIES})

add_executable(test_simulated_references src/test_simulated_references.cpp )
add_dependencies(test_simulated_references ${PROJECT_NAME}_gencpp)
target_link_libraries(test_simulated_references ${catkin_LIBRARIES} kdl_parser ${orocos_kdl_LIBRARIES})


add_executable(test_parse_list_yaml src/test_parse_list_yaml.cpp)
add_dependencies(test_parse_list_yaml test_parse_list_yaml_gencpp ${PROJECT_NAME})
target_link_libraries(test_parse_list_yaml ${PROJECT_NAME} ${catkin_LIBRARIES} kdl_parser ${orocos_kdl_LIBRARIES})



SET( STATES_FILES
src/Grasp_obj_state.cpp
src/Home_State_grafo.cpp
src/Pos_trash_state.cpp
src/SoftHand_close_state.cpp
src/SoftHand_open_state.cpp
src/Exit_state.cpp
src/Wait_msg_state.cpp
src/Remove_obj_state.cpp
src/Overtune_state.cpp
src/HandPoseGenerator_state.cpp
)

add_executable(test_grafo ${STATES_FILES} src/desperate_mind_state_machine.cpp src/main_grafo.cpp)
add_dependencies(test_grafo test_grafo_generate_messages_cpp )
target_link_libraries(test_grafo ${catkin_LIBRARIES}  kdl_parser ${orocos_kdl_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES desperate_housewife_controllers.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})