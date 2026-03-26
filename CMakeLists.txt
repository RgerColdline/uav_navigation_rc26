cmake_minimum_required(VERSION 2.8.3)
project(uav_navigation)

## Compile as C++14 (required for PCL 1.10+)
add_compile_options(-std=c++14)

## Find catkin macros and libraries
# 【修改 1】必须加上 tf 依赖！因为 demo_fsm 里面物理解算用到了 tf
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  mavros
  mavros_msgs
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  quadrotor_msgs
  tf   
)

## System dependencies
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
# 【修改 2】把 tf 加到这里
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs mavros mavros_msgs sensor_msgs nav_msgs pcl_ros pcl_conversions quadrotor_msgs tf
  DEPENDS PCL
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ==========================================================
# 1. 核心模块 A：点云拉伸节点 (给 Ego-planner 喂数据)
# ==========================================================
add_executable(cloud_extruder src/cloud_extruder.cpp)
target_link_libraries(cloud_extruder ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# ==========================================================
# 2. 核心模块 B：避障执行翻译节点 (黑盒输出，平滑飞控)
# ==========================================================
add_executable(ego_controller_node src/ego_controller_node.cpp)
target_link_libraries(ego_controller_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# ==========================================================
# 3. 测试示例模块 C：FSM Demo (交给队友 B 参考的范例)
# ==========================================================
add_executable(demo_fsm_node src/main_fsm.cpp)
target_link_libraries(demo_fsm_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})


#############
## Install ##
#############

## Mark executables for installation
install(TARGETS ego_controller_node cloud_extruder demo_fsm_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Mark Python scripts for installation (如果还有的话)
catkin_install_python(PROGRAMS
  scripts/test_send_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)