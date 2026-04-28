cmake_minimum_required(VERSION 3.0.2)
project(tl_optional)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)