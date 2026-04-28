cmake_minimum_required(VERSION 3.0.2)
project(dm_arm_msgs)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
    actionlib_msgs
)

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_service_files(
    FILES
    CommandEef.srv
    ConfigArm.srv
    QueryArm.srv
    CommandOctomap.srv
)

add_action_files(
    FILES
    PickTask.action
    MoveArm.action
    SimpleMoveArm.action
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
    actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS 
    geometry_msgs 
    std_msgs 
    actionlib_msgs
    message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
