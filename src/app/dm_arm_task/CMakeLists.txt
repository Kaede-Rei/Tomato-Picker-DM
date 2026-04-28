cmake_minimum_required(VERSION 3.0.2)

# ========================= 项目配置(必填) ========================= #
# TODO: 修改项目名称，必须与 package.xml 中的 <name> 一致
project(dm_arm_task)

# ========================= 基本配置 ========================= #
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================= 依赖配置(按需修改) ========================= #
# TODO: 在这里填写所有 catkin 依赖；如果依赖你自己的接口包，也写在这里
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  dm_arm_msgs
  dm_arm_controller
  roscpp
  tl_expected
  dm_arm_acm_guard
)

# TODO: 如需第三方库，在这里手动添加
# 例如：find_package(Eigen3 REQUIRED)
# 例如：find_package(OpenCV REQUIRED)

# ========================= catkin 导出配置 ========================= #
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    dm_arm_msgs
    dm_arm_controller
    roscpp
    tl_expected
    dm_arm_acm_guard
  DEPENDS
    # Eigen3
    # OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${EIGEN3_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
)

# ========================= 共享库配置(可选) ========================= #
# TODO: 若无公共库，保持 LIB_SOURCES 为空即可
set(LIB_NAME ${PROJECT_NAME})
set(LIB_SOURCES
  src/tasks_manager.cpp
  src/pick_action.cpp
)

if(LIB_SOURCES)
  add_library(${LIB_NAME} ${LIB_SOURCES})
  add_dependencies(${LIB_NAME} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${LIB_NAME}
    ${catkin_LIBRARIES}
    # Eigen3::Eigen
    # ${OpenCV_LIBS}
  )
endif()

# ========================= Python 脚本安装(可选) ========================= #
# TODO: 若无 Python 脚本，保持为空即可
set(PYTHON_SCRIPTS
  # scripts/demo_node.py
  # scripts/tool.py
)

if(PYTHON_SCRIPTS)
  catkin_install_python(PROGRAMS
    ${PYTHON_SCRIPTS}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

# ========================= 安装配置 ========================= #
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
elseif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include)
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

# TODO: 自动安装常见资源目录；按需增删
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
  # TODO: 如需 rostest / gtest，可在这里补充
  # find_package(rostest REQUIRED)
  # add_rostest(test/example.test)
  # catkin_add_gtest(test_example test/test_example.cpp)
endif()
