cmake_minimum_required(VERSION 3.1.1)
project(localization_informed_planning_sim)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs genmsg actionlib_msgs geometry_msgs)

add_action_files(
  DIRECTORY action
  FILES
  MoveToPosition.action
  AttemptGrasp.action
)

#
add_service_files(
  FILES
  SetControllerTarget.srv
  SetGlobalPosition.srv
)

add_message_files(
  FILES
  GlobalPositionControllerState.msg
  BreadcrumbLocalizationResult.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs actionlib_msgs geometry_msgs)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch robots
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
