cmake_minimum_required(VERSION 3.0.2)
project(template)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 查找必要的包
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  nav_msgs
  mavros_msgs
  message_generation
  cv_bridge
  image_transport
  std_srvs
)

find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")


# 添加消息生成依赖（如果需要自定义消息）
# add_message_files(
#   FILES
#   # Command.msg
# )

# 生成消息（如果需要）
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   sensor_msgs
#   std_srvs
# )

# Catkin包配置
catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES 
    template
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs
    std_srvs
    sensor_msgs 
    geometry_msgs
    tf
    nav_msgs
    mavros_msgs
    cv_bridge
    image_transport
  DEPENDS 
    EIGEN3
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 编译主程序（包含避障功能）
add_executable(template 
  src/template.cpp
  src/collision_avoidance.cpp
  src/gate_crossing.cpp
)

# 链接库
target_link_libraries(template 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# 安装目标
install(TARGETS
  template
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# 安装launch文件（如果有）
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

# 安装配置文件（如果有）
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)
