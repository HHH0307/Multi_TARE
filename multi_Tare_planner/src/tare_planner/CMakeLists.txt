cmake_minimum_required(VERSION 3.5)
project(tare_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# 生成消息和服务（ROS 2核心机制）
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RequestPath.srv"
  "msg/Subnode.msg"
  "msg/Subgraph.msg"
  "msg/ReturnHome.msg"
  "msg/SharedNode.msg"
  "msg/SharedEdge.msg"
  "msg/SharedMergerInfor.msg"
  "msg/MergerEdge.msg"  
  "msg/MergerGraphEdge.msg"

  DEPENDENCIES std_msgs geometry_msgs nav_msgs  # 依赖的其他消息包
)

 include_directories(
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  include
  or-tools/include
  ${EIGEN3_INCLUDE_DIR}
  /opt/ros/humble/include
)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

add_library(lidar_model src/lidar_model/lidar_model.cpp)
ament_target_dependencies(lidar_model rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 visualization_msgs pcl_ros pcl_conversions)
# target_link_libraries(lidar_model ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(graph src/graph/graph.cpp)
ament_target_dependencies(graph rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)
# target_link_libraries(graph ${catkin_LIBRARIES})

add_library(misc_utils src/utils/misc_utils.cpp)
ament_target_dependencies(misc_utils rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)
# target_link_libraries(misc_utils ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(pointcloud_utils src/utils/pointcloud_utils.cpp)
ament_target_dependencies(pointcloud_utils rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)
# target_link_libraries(pointcloud_utils ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(tsp_solver src/tsp_solver/tsp_solver.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/Allocation_strategy/Allocation_strategy.cpp)
ament_target_dependencies(tsp_solver rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(tsp_solver ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/or-tools/lib/libortools.so)

add_library(viewpoint src/viewpoint/viewpoint.cpp)
ament_target_dependencies(viewpoint rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(viewpoint ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} lidar_model)

add_library(rolling_grid src/rolling_grid/rolling_grid.cpp)
ament_target_dependencies(rolling_grid rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)
target_link_libraries(rolling_grid ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} misc_utils pointcloud_utils)

add_library(viewpoint_manager STATIC src/viewpoint_manager/viewpoint_manager.cpp)
ament_target_dependencies(viewpoint_manager rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(viewpoint_manager ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} rolling_grid viewpoint grid_world)

add_library(local_coverage_planner src/local_coverage_planner/local_coverage_planner.cpp)
ament_target_dependencies(local_coverage_planner rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(local_coverage_planner ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} viewpoint_manager planning_env)

add_library(grid_world STATIC src/grid_world/grid_world.cpp)
ament_target_dependencies(grid_world rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(grid_world ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} viewpoint_manager tsp_solver keypose_graph exploration_path merger_graph)

add_library(pointcloud_manager src/pointcloud_manager/pointcloud_manager.cpp)
ament_target_dependencies(pointcloud_manager rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)
target_link_libraries(pointcloud_manager ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} misc_utils pointcloud_utils)

add_library(keypose_graph STATIC src/keypose_graph/keypose_graph.cpp)
ament_target_dependencies(keypose_graph rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(keypose_graph ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} viewpoint_manager planning_env)

# 新增
add_library(merger_graph STATIC src/merger_graph/merger_graph.cpp)
ament_target_dependencies(merger_graph rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)
target_link_libraries(merger_graph ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} viewpoint_manager keypose_graph)

add_library(rolling_occupancy_grid src/rolling_occupancy_grid/rolling_occupancy_grid.cpp)
ament_target_dependencies(rolling_occupancy_grid rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)
target_link_libraries(rolling_occupancy_grid ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} rolling_grid misc_utils)

add_library(planning_env STATIC src/planning_env/planning_env.cpp)
ament_target_dependencies(planning_env rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(planning_env ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} rolling_occupancy_grid pointcloud_manager lidar_model viewpoint_manager pointcloud_utils misc_utils)

add_library(exploration_path src/exploration_path/exploration_path.cpp)
ament_target_dependencies(exploration_path rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)
target_link_libraries(exploration_path ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(tare_visualizer src/tare_visualizer/tare_visualizer.cpp)
ament_target_dependencies(tare_visualizer rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)
target_link_libraries(tare_visualizer ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} grid_world misc_utils)

# 新增
add_library(multi_tare_visualizer src/tare_visualizer/multi_tare_visualizer.cpp)
ament_target_dependencies(multi_tare_visualizer rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)
target_link_libraries(multi_tare_visualizer ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} grid_world misc_utils)

add_library(sensor_coverage_planner_ground src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp)
ament_target_dependencies(sensor_coverage_planner_ground rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(sensor_coverage_planner_ground ${catkin_LIBRARIES} graph planning_env keypose_graph viewpoint_manager pointcloud_manager grid_world local_coverage_planner tare_visualizer)

# 新增
add_library(multi_sensor_coverage_planner_ground src/sensor_coverage_planner/multi_sensor_coverage_planner_ground.cpp)
ament_target_dependencies(multi_sensor_coverage_planner_ground rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(multi_sensor_coverage_planner_ground ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} graph planning_env keypose_graph merger_graph viewpoint_manager pointcloud_manager grid_world local_coverage_planner multi_tare_visualizer)

add_executable(navigationBoundary src/navigation_boundary_publisher/navigationBoundary.cpp)
ament_target_dependencies(navigationBoundary rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions)
target_link_libraries(navigationBoundary ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(tare_planner_node src/tare_planner_node/tare_planner_node.cpp)
ament_target_dependencies(tare_planner_node rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(tare_planner_node ${catkin_LIBRARIES} sensor_coverage_planner_ground)

# 新增
add_executable(multi_tare_planner_node src/tare_planner_node/multi_tare_planner_node.cpp)
ament_target_dependencies(multi_tare_planner_node rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(multi_tare_planner_node ${PROJECT_NAME}__rosidl_typesupport_cpp ${catkin_LIBRARIES} multi_sensor_coverage_planner_ground)

#############
## Install ##
#############

install(TARGETS
  navigationBoundary
  tare_planner_node
  multi_tare_planner_node
  DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/
        )
install(DIRECTORY rviz/
        DESTINATION share/${PROJECT_NAME}/
        )
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}
      )
install(DIRECTORY data/
        DESTINATION share/${PROJECT_NAME}
      )
install(DIRECTORY msg/
        DESTINATION share/${PROJECT_NAME}/msg
        )
install(DIRECTORY srv/
        DESTINATION share/${PROJECT_NAME}/srv
        )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 新增
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(geometry_msgs)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)

ament_package()