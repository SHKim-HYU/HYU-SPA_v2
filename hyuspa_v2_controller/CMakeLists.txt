cmake_minimum_required(VERSION 2.8.3)

project(hyuspa_v2_controller)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
  angles
  controller_interface
  control_msgs
  urdf
  kdl_parser
  realtime_tools
  dynamic_reconfigure
)
find_package(Eigen3 REQUIRED NO_MODULES)
# message
add_message_files(FILES ControllerJointState.msg)
generate_messages(DEPENDENCIES std_msgs)

# include
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    angles
    controller_interface
    control_msgs
    urdf
    kdl_parser
    realtime_tools
    dynamic_reconfigure
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/computedtorque_control_clik.cpp
         include/Controller.cpp
         include/LieOperator.cpp
         include/LieDynamics.cpp
         include/PoEKinematics.cpp
         include/PropertyDefinition.h
         include/Trajectory.cpp
	 include/SerialRobot.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

