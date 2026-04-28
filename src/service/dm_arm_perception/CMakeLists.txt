cmake_minimum_required(VERSION 3.0.2)

# ========================= 项目配置(必填) ========================= #
# TODO: 修改项目名称，必须与 package.xml 中的 <name> 一致
project(dm_arm_perception)

# ========================= 基本配置 ========================= #
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================= 依赖配置(按需修改) ========================= #
# TODO: 在这里填写所有依赖，会自动同步到 find_package 和 catkin_package
set(DEPS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  message_filters
  pcl_ros
  pcl_conversions
  tf2_ros
  tf2_sensor_msgs
  dm_arm_msgs
)

find_package(catkin REQUIRED COMPONENTS ${DEPS})

# TODO: 如需第三方库，在这里手动添加
# 例如：find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io octree segmentation surface)

# ========================= catkin 导出配置 ========================= #
# 自动判断 include 目录是否存在，防止 catkin_package 报错
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include)
  set(EXPORT_INCLUDE_DIRS include)
  include_directories(include)
else()
  set(EXPORT_INCLUDE_DIRS "")
endif()

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS ${EXPORT_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${DEPS}
  DEPENDS
    # Eigen3
)

# ========================= 共享库配置(可选) ========================= #
# TODO: 若无公共库，保持 LIB_SOURCES 为空即可
set(LIB_NAME ${PROJECT_NAME})
set(LIB_SOURCES
  # src/library.cpp
)

if(LIB_SOURCES)
  add_library(${LIB_NAME} ${LIB_SOURCES})
  target_link_libraries(${LIB_NAME}
    ${catkin_LIBRARIES}
  )
endif()

# ========================= C++ 节点配置 ========================= #
# TODO: 在此列表中添加所有节点名称(不带 .cpp 后缀)
set(NODES
  # node_name
  cloud_preprocessor
)

foreach(node IN LISTS NODES)
  if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/${node}.cpp)
    message(FATAL_ERROR "节点源码文件不存在：src/${node}.cpp")
  endif()

  add_executable(${node} src/${node}.cpp)
  target_link_libraries(${node}
    ${catkin_LIBRARIES}
  )

  if(LIB_SOURCES)
    target_link_libraries(${node} ${LIB_NAME})
  endif()

  # 自动添加消息/服务生成的依赖
  add_dependencies(${node} ${catkin_EXPORTED_TARGETS})
  if(TARGET ${${PROJECT_NAME}_EXPORTED_TARGETS})
    add_dependencies(${node} ${${PROJECT_NAME}_EXPORTED_TARGETS})
  endif()
endforeach()

# ========================= Python 脚本安装(可选) ========================= #
set(PYTHON_SCRIPTS
  # scripts/demo_node.py
)

if(PYTHON_SCRIPTS)
  catkin_install_python(PROGRAMS
    ${PYTHON_SCRIPTS}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

# ========================= 安装配置 ========================= #
if(EXPORT_INCLUDE_DIRS)
  install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
endif()

if(LIB_SOURCES)
  install(TARGETS ${LIB_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
endif()

if(NODES)
  install(TARGETS ${NODES}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

# TODO: 自动安装常见资源目录
foreach(dir IN ITEMS launch config params urdf meshes textures rviz srdf worlds maps)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(
      DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
    )
  endif()
endforeach()

# ========================= 测试配置(可选) ========================= #
if(CATKIN_ENABLE_TESTING)
  # find_package(rostest REQUIRED)
endif()
