cmake_minimum_required(VERSION 3.0.2)
project(dm_hw)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  controller_manager
  dm_arm_msgs
  hardware_interface
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    control_msgs
    controller_manager
    dm_arm_msgs
    hardware_interface
    roscpp
    sensor_msgs
    std_msgs
    trajectory_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/clangd_provider.cpp
  src/dm_hardware_interface.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(dm_controller src/dm_controller.cpp)
add_dependencies(dm_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(dm_controller
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(test_damiao src/test_damiao.cpp)
target_link_libraries(test_damiao
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} dm_controller test_damiao
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
