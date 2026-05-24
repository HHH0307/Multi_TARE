#include "sensor_coverage_planner/multi_sensor_coverage_planner_ground.h"
#include "graph/graph.h"
#include <memory>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

namespace sensor_coverage_planner_3d_ns {

void SensorCoveragePlanner3D::ReadParameters() {
  this->declare_parameter<std::string>("robot_name", "robot_0"); // 新增 机器人名字
  this->declare_parameter<std::string>("allocation_strategy_", "Mdvrp"); // 新增 分配策略
  this->declare_parameter<std::string>("robot_public_name_", "robot");
  this->declare_parameter<std::string>("sub_start_exploration_topic_", "/exploration_start");
  this->declare_parameter<std::string>("sub_state_estimation_topic_", "/state_estimation_at_scan");
  this->declare_parameter<std::string>("sub_registered_scan_topic_", "/registered_scan");
  this->declare_parameter<std::string>("sub_terrain_map_topic_", "/terrain_map");
  this->declare_parameter<std::string>("sub_terrain_map_ext_topic_", "/terrain_map_ext");
  this->declare_parameter<std::string>("sub_coverage_boundary_topic_", "/coverage_boundary");
  this->declare_parameter<std::string>("sub_viewpoint_boundary_topic_", "/navigation_boundary");
  this->declare_parameter<std::string>("sub_nogo_boundary_topic_", "/nogo_boundary");
  this->declare_parameter<std::string>("sub_joystick_topic_", "/joy");
  this->declare_parameter<std::string>("sub_reset_waypoint_topic_", "/reset_waypoint");
  this->declare_parameter<std::string>("pub_exploration_finish_topic_", "exploration_finish");
  this->declare_parameter<std::string>("pub_runtime_breakdown_topic_", "runtime_breakdown");
  this->declare_parameter<std::string>("pub_runtime_topic_", "/runtime");
  this->declare_parameter<std::string>("pub_waypoint_topic_", "/way_point");
  this->declare_parameter<std::string>("pub_momentum_activation_count_topic_", "momentum_activation_count");

  // Bool
  this->declare_parameter<bool>("kAutoStart", false);
  this->declare_parameter<bool>("kRushHome", false);
  this->declare_parameter<bool>("kUseTerrainHeight", true);
  this->declare_parameter<bool>("kCheckTerrainCollision", true);
  this->declare_parameter<bool>("kExtendWayPoint", true);
  this->declare_parameter<bool>("kUseLineOfSightLookAheadPoint", true);
  this->declare_parameter<bool>("kNoExplorationReturnHome", true);
  this->declare_parameter<bool>("kUseMomentum", false);

  // Double
  this->declare_parameter<double>("kKeyposeCloudDwzFilterLeafSize", 0.2);
  this->declare_parameter<double>("kRushHomeDist", 10.0);
  this->declare_parameter<double>("kAtHomeDistThreshold", 0.5);
  this->declare_parameter<double>("kTerrainCollisionThreshold", 0.5);
  this->declare_parameter<double>("kLookAheadDistance", 5.0);
  this->declare_parameter<double>("kExtendWayPointDistanceBig", 8.0);
  this->declare_parameter<double>("kExtendWayPointDistanceSmall", 3.0);

  // Int
  this->declare_parameter<int>("kDirectionChangeCounterThr", 4);
  this->declare_parameter<int>("kDirectionNoChangeCounterThr", 5);
  this->declare_parameter<int>("kResetWaypointJoystickAxesID", 0);
  this->declare_parameter<int>("robot_num", 3);  // 新增 robot_num 声明

  // grid_world
  this->declare_parameter<int>("kGridWorldXNum", 121);
  this->declare_parameter<int>("kGridWorldYNum", 121);
  this->declare_parameter<int>("kGridWorldZNum", 121);
  this->declare_parameter<double>("kGridWorldCellHeight", 8.0);
  this->declare_parameter<int>("kGridWorldNearbyGridNum", 5);
  this->declare_parameter<int>("kMinAddPointNumSmall", 60);
  this->declare_parameter<int>("kMinAddPointNumBig", 100);
  this->declare_parameter<int>("kMinAddFrontierPointNum", 30);
  this->declare_parameter<int>("kCellExploringToCoveredThr", 1);
  this->declare_parameter<int>("kCellCoveredToExploringThr", 10);
  this->declare_parameter<int>("kCellExploringToAlmostCoveredThr", 10);
  this->declare_parameter<int>("kCellAlmostCoveredToExploringThr", 20);
  this->declare_parameter<int>("kCellUnknownToExploringThr", 1);

  // keypose_graph
  this->declare_parameter<double>("keypose_graph/kAddNodeMinDist", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddNonKeyposeNodeMinDist", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeConnectDistThr", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeToLastKeyposeDistThr", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeVerticalThreshold", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeCollisionCheckResolution", 0.5);
  this->declare_parameter<double>("keypose_graph/kAddEdgeCollisionCheckRadius", 0.5);
  this->declare_parameter<int>("keypose_graph/kAddEdgeCollisionCheckPointNumThr", 1);

  // local_coverage_planner
  this->declare_parameter<int>("kGreedyViewPointSampleRange", 5);
  this->declare_parameter<int>("kLocalPathOptimizationItrMax", 10);

  // planning_env
  this->declare_parameter<double>("kSurfaceCloudDwzLeafSize", 0.2);
  this->declare_parameter<double>("kCollisionCloudDwzLeafSize", 0.2);
  this->declare_parameter<int>("kKeyposeCloudStackNum", 5);
  this->declare_parameter<int>("kPointCloudRowNum", 20);
  this->declare_parameter<int>("kPointCloudColNum", 20);
  this->declare_parameter<int>("kPointCloudLevelNum", 10);
  this->declare_parameter<int>("kMaxCellPointNum", 100000);
  this->declare_parameter<double>("kPointCloudCellSize", 24.0);
  this->declare_parameter<double>("kPointCloudCellHeight", 3.0);
  this->declare_parameter<int>("kPointCloudManagerNeighborCellNum", 5);
  this->declare_parameter<double>("kCoverCloudZSqueezeRatio", 2.0);
  this->declare_parameter<double>("kFrontierClusterTolerance", 1.0);
  this->declare_parameter<int>("kFrontierClusterMinSize", 30);
  this->declare_parameter<bool>("kUseCoverageBoundaryOnFrontier", false);
  this->declare_parameter<bool>("kUseCoverageBoundaryOnObjectSurface", false);

  // rolling_occupancy_grid
  this->declare_parameter<double>("rolling_occupancy_grid/resolution_x", 0.3);
  this->declare_parameter<double>("rolling_occupancy_grid/resolution_y", 0.3);
  this->declare_parameter<double>("rolling_occupancy_grid/resolution_z", 0.3);

  // viewpoint_manager
  this->declare_parameter<int>("viewpoint_manager/number_x", 50);
  this->declare_parameter<int>("viewpoint_manager/number_y", 80);
  this->declare_parameter<int>("viewpoint_manager/number_z", 40);
  this->declare_parameter<double>("viewpoint_manager/resolution_x", 0.5);
  this->declare_parameter<double>("viewpoint_manager/resolution_y", 0.5);
  this->declare_parameter<double>("viewpoint_manager/resolution_z", 0.5);
  this->declare_parameter<double>("kConnectivityHeightDiffThr", 0.25);
  this->declare_parameter<double>("kViewPointCollisionMargin", 0.5);
  this->declare_parameter<double>("kViewPointCollisionMarginZPlus", 0.5);
  this->declare_parameter<double>("kViewPointCollisionMarginZMinus", 0.5);
  this->declare_parameter<double>("kCollisionGridZScale", 2.0);
  this->declare_parameter<double>("kCollisionGridResolutionX", 0.5);
  this->declare_parameter<double>("kCollisionGridResolutionY", 0.5);
  this->declare_parameter<double>("kCollisionGridResolutionZ", 0.5);
  this->declare_parameter<bool>("kLineOfSightStopAtNearestObstacle", true);
  this->declare_parameter<bool>("kCheckDynamicObstacleCollision", true);
  this->declare_parameter<int>("kCollisionFrameCountMax", 3);
  this->declare_parameter<double>("kViewPointHeightFromTerrain", 0.75);
  this->declare_parameter<double>("kViewPointHeightFromTerrainChangeThreshold", 0.6);
  this->declare_parameter<int>("kCollisionPointThr", 3);
  this->declare_parameter<double>("kCoverageOcclusionThr", 1.0);
  this->declare_parameter<double>("kCoverageDilationRadius", 1.0);
  this->declare_parameter<double>("kCoveragePointCloudResolution", 1.0);
  this->declare_parameter<double>("kSensorRange", 10.0);
  this->declare_parameter<double>("kNeighborRange", 3.0);

  // tare_visualizer
  this->declare_parameter<bool>("kExploringSubspaceMarkerColorGradientAlpha", true);
  this->declare_parameter<double>("kExploringSubspaceMarkerColorMaxAlpha", 1.0);
  this->declare_parameter<double>("kExploringSubspaceMarkerColorR", 0.0);
  this->declare_parameter<double>("kExploringSubspaceMarkerColorG", 1.0);
  this->declare_parameter<double>("kExploringSubspaceMarkerColorB", 0.0);
  this->declare_parameter<double>("kExploringSubspaceMarkerColorA", 1.0);
  this->declare_parameter<double>("kLocalPlanningHorizonMarkerColorR", 0.0);
  this->declare_parameter<double>("kLocalPlanningHorizonMarkerColorG", 1.0);
  this->declare_parameter<double>("kLocalPlanningHorizonMarkerColorB", 0.0);
  this->declare_parameter<double>("kLocalPlanningHorizonMarkerColorA", 1.0);
  this->declare_parameter<double>("kLocalPlanningHorizonMarkerWidth", 0.3);
  this->declare_parameter<double>("kLocalPlanningHorizonHeight", 3.0);

  this->get_parameter("robot_name", robot_name);
  this->get_parameter("allocation_strategy_", allocation_strategy_);
  this->get_parameter("robot_num", robot_num);             //获得当前机器人个数
  this->get_parameter("robot_public_name_", robot_public_name_);

  bool got_parameter = true;
  got_parameter &= this->get_parameter("sub_start_exploration_topic_", sub_start_exploration_topic_);
  if (!got_parameter) {
    std::cout << "Failed to get parameter sub_start_exploration_topic_" << std::endl;
  }
  this->get_parameter("sub_state_estimation_topic_", sub_state_estimation_topic_);
  this->get_parameter("sub_registered_scan_topic_", sub_registered_scan_topic_);
  this->get_parameter("sub_terrain_map_topic_", sub_terrain_map_topic_);
  this->get_parameter("sub_terrain_map_ext_topic_", sub_terrain_map_ext_topic_);
  this->get_parameter("sub_coverage_boundary_topic_", sub_coverage_boundary_topic_);
  this->get_parameter("sub_viewpoint_boundary_topic_", sub_viewpoint_boundary_topic_);
  this->get_parameter("sub_nogo_boundary_topic_", sub_nogo_boundary_topic_);
  this->get_parameter("sub_joystick_topic_", sub_joystick_topic_);
  this->get_parameter("sub_reset_waypoint_topic_", sub_reset_waypoint_topic_);
  this->get_parameter("pub_exploration_finish_topic_", pub_exploration_finish_topic_);
  this->get_parameter("pub_runtime_breakdown_topic_", pub_runtime_breakdown_topic_);
  this->get_parameter("pub_runtime_topic_", pub_runtime_topic_);
  this->get_parameter("pub_waypoint_topic_", pub_waypoint_topic_);
  this->get_parameter("pub_momentum_activation_count_topic_", pub_momentum_activation_count_topic_);

  this->get_parameter("kAutoStart", kAutoStart);
  std::cout << "parameter kAutoStart: " << kAutoStart << std::endl;

  this->get_parameter("kRushHome", kRushHome);
  this->get_parameter("kUseTerrainHeight", kUseTerrainHeight);
  this->get_parameter("kCheckTerrainCollision", kCheckTerrainCollision);
  this->get_parameter("kExtendWayPoint", kExtendWayPoint);
  this->get_parameter("kUseLineOfSightLookAheadPoint", kUseLineOfSightLookAheadPoint);
  this->get_parameter("kNoExplorationReturnHome", kNoExplorationReturnHome);
  this->get_parameter("kUseMomentum", kUseMomentum);
  this->get_parameter("kKeyposeCloudDwzFilterLeafSize", kKeyposeCloudDwzFilterLeafSize);
  this->get_parameter("kRushHomeDist", kRushHomeDist);
  this->get_parameter("kAtHomeDistThreshold", kAtHomeDistThreshold);
  this->get_parameter("kTerrainCollisionThreshold", kTerrainCollisionThreshold);
  this->get_parameter("kLookAheadDistance", kLookAheadDistance);
  this->get_parameter("kExtendWayPointDistanceBig", kExtendWayPointDistanceBig);
  this->get_parameter("kExtendWayPointDistanceSmall", kExtendWayPointDistanceSmall);
  this->get_parameter("kDirectionChangeCounterThr", kDirectionChangeCounterThr);
  this->get_parameter("kDirectionNoChangeCounterThr", kDirectionNoChangeCounterThr);
  this->get_parameter("kResetWaypointJoystickAxesID", kResetWaypointJoystickAxesID);

  // 新增
  int Local_planning_x_num = this->get_parameter("viewpoint_manager/number_x").as_int();
  double Local_planning_resolution_x = this->get_parameter("viewpoint_manager/resolution_x").as_double();
  kLocal_planning_range = Local_planning_x_num * Local_planning_resolution_x; // 50 * 0.6 = 30

  // 获取  网格单元大小
  int viewpoint_number = this->get_parameter("viewpoint_manager/number_x").as_int();  //  indoor     50
  double viewpoint_resolution = this->get_parameter("viewpoint_manager/resolution_x").as_double();  // indoor  0.6
  kCellSize_ = viewpoint_number * viewpoint_resolution / 5;  // 50 * 0.6 / 5 = 6

  for(int i = 0; i < robot_num; i++)
  {
    is_returning_home_list.push_back(false);
  }

}

void SensorCoveragePlanner3D::InitializeData() {
  keypose_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
          shared_from_this(), "keypose_cloud", robot_name+"/"+kWorldFrameID);
  registered_scan_stack_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>>(
          shared_from_this(), "registered_scan_stack", robot_name+"/"+kWorldFrameID);
  registered_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "registered_cloud", robot_name+"/"+kWorldFrameID);
  large_terrain_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "terrain_cloud_large", robot_name+"/"+kWorldFrameID);
  terrain_collision_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "terrain_collision_cloud", robot_name+"/"+kWorldFrameID);
  terrain_ext_collision_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "terrain_ext_collision_cloud", robot_name+"/"+kWorldFrameID);
  viewpoint_vis_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "viewpoint_vis_cloud", robot_name+"/"+kWorldFrameID);
  grid_world_vis_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "grid_world_vis_cloud", robot_name+"/"+kWorldFrameID);
  exploration_path_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "bspline_path_cloud", robot_name+"/"+kWorldFrameID);

  selected_viewpoint_vis_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "selected_viewpoint_vis_cloud", robot_name+"/"+kWorldFrameID);
  exploring_cell_vis_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "exploring_cell_vis_cloud", robot_name+"/"+kWorldFrameID);
  collision_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "collision_cloud", robot_name+"/"+kWorldFrameID);
  lookahead_point_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "lookahead_point_cloud", robot_name+"/"+kWorldFrameID);
  keypose_graph_vis_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "keypose_graph_cloud", robot_name+"/"+kWorldFrameID);
  // 新增
  merger_graph_vis_cloud_ = 
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "merger_graph_cloud", robot_name+"/"+kWorldFrameID);
  viewpoint_in_collision_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "viewpoint_in_collision_cloud_", robot_name+"/"+kWorldFrameID);
  point_cloud_manager_neighbor_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "pointcloud_manager_cloud", robot_name+"/"+kWorldFrameID);
  reordered_global_subspace_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
          shared_from_this(), "reordered_global_subspace_cloud", robot_name+"/"+kWorldFrameID);

  //新增可视化点云
  MTSP_grid_graph_vis_cloud_ = 
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(  // MTSP_grid_graph 点云
          shared_from_this(), "MTSP_grid_graph_cloud", robot_name+"/"+kWorldFrameID);

  viewpoint_manager_ = std::make_shared<viewpoint_manager_ns::ViewPointManager>(shared_from_this());
  // 初始化关键变量
  planning_env_ = std::make_shared<planning_env_ns::PlanningEnv>(shared_from_this(), robot_name + "/" + kWorldFrameID);  //规划  环境
  // planning_env_ = std::make_shared<planning_env_ns::PlanningEnv>(shared_from_this());

  local_coverage_planner_ = std::make_shared<local_coverage_planner_ns::LocalCoveragePlanner>(shared_from_this());
  local_coverage_planner_->SetViewPointManager(viewpoint_manager_);
  keypose_graph_ = std::make_shared<keypose_graph_ns::KeyposeGraph>(shared_from_this());
  merger_graph_ = std::make_shared<merger_graph_ns::MergerGraph>(shared_from_this());  //新增 关键位姿图

  grid_world_ = std::make_shared<grid_world_ns::GridWorld>(shared_from_this());
  grid_world_->SetUseKeyposeGraph(true);
  visualizer_ = std::make_shared<tare_visualizer_ns::TAREVisualizer>(shared_from_this());

  initial_position_.x() = 0.0;
  initial_position_.y() = 0.0;
  initial_position_.z() = 0.0;

  cur_keypose_node_ind_ = 0;

  keypose_graph_node_marker_ = std::make_shared<misc_utils_ns::Marker>(shared_from_this(), "keypose_graph_node_marker", robot_name+"/"+kWorldFrameID);
  keypose_graph_node_marker_->SetType(visualization_msgs::msg::Marker::POINTS);
  keypose_graph_node_marker_->SetScale(0.4, 0.4, 0.1);
  keypose_graph_node_marker_->SetColorRGBA(1.0, 0.0, 0.0, 1.0);
  
  keypose_graph_edge_marker_ = std::make_shared<misc_utils_ns::Marker>(shared_from_this(), "keypose_graph_edge_marker", robot_name+"/"+kWorldFrameID);
  keypose_graph_edge_marker_->SetType(visualization_msgs::msg::Marker::LINE_LIST);
  keypose_graph_edge_marker_->SetScale(0.05, 0.0, 0.0);
  keypose_graph_edge_marker_->SetColorRGBA(1.0, 1.0, 0.0, 0.9);

  // 新增  merger_graph_  的  可视化   参照 keypose_graph_
  // 拼接图   节点标记
  merger_graph_node_marker_ = std::make_shared<misc_utils_ns::Marker>(shared_from_this(), "merger_graph_node_marker", robot_name+"/"+kWorldFrameID);
  merger_graph_node_marker_->SetType(visualization_msgs::msg::Marker::POINTS);
  merger_graph_node_marker_->SetScale(0.4, 0.4, 0.1);
  merger_graph_node_marker_->SetColorRGBA(1.0, 0.0, 0.0, 1.0);
  //拼接图图  边标记
  merger_graph_edge_marker_ = std::make_shared<misc_utils_ns::Marker>(shared_from_this(), "merger_graph_edge_marker", robot_name+"/"+kWorldFrameID);
  merger_graph_edge_marker_->SetType(visualization_msgs::msg::Marker::LINE_LIST);
  merger_graph_edge_marker_->SetScale(0.05, 0.0, 0.0);
  merger_graph_edge_marker_->SetColorRGBA(1.0, 0.0, 1.0, 0.9); //作出区别

  nogo_boundary_marker_ = std::make_shared<misc_utils_ns::Marker>(shared_from_this(), "nogo_boundary_marker", robot_name+"/"+kWorldFrameID);
  nogo_boundary_marker_->SetType(visualization_msgs::msg::Marker::LINE_LIST);
  nogo_boundary_marker_->SetScale(0.05, 0.0, 0.0);
  nogo_boundary_marker_->SetColorRGBA(1.0, 0.0, 0.0, 0.8);

  grid_world_marker_ = std::make_shared<misc_utils_ns::Marker>(shared_from_this(), "grid_world_marker", robot_name+"/"+kWorldFrameID);
  grid_world_marker_->SetType(visualization_msgs::msg::Marker::CUBE_LIST);
  grid_world_marker_->SetScale(1.0, 1.0, 1.0);
  grid_world_marker_->SetColorRGBA(1.0, 0.0, 0.0, 0.8);

  // 新增可视化  关于MTSP 网格图   MTSP_grid_graph
  // MTSP_grid_graph  图  结点可视化
  MTSP_grid_graph_node_marker_ = std::make_shared<misc_utils_ns::Marker>(shared_from_this(), "MTSP_grid_graph_node_marker", robot_name+"/"+kWorldFrameID);
  MTSP_grid_graph_node_marker_->SetType(visualization_msgs::msg::Marker::POINTS);
  MTSP_grid_graph_node_marker_->SetScale(0.4, 0.4, 0.1);
  MTSP_grid_graph_node_marker_->SetColorRGBA(1.0, 0.0, 0.0, 1.0);
  // MTSP_grid_graph 图  边可视化
  MTSP_grid_graph_edge_marker_ = std::make_shared<misc_utils_ns::Marker>(shared_from_this(), "MTSP_grid_graph_edge_marker", robot_name+"/"+kWorldFrameID);
  MTSP_grid_graph_edge_marker_->SetType(visualization_msgs::msg::Marker::LINE_LIST);
  MTSP_grid_graph_edge_marker_->SetScale(0.05, 0.0, 0.0);
  MTSP_grid_graph_edge_marker_->SetColorRGBA(1.0, 1.0, 0.0, 0.9);

  robot_yaw_ = 0.0;
  lookahead_point_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
  moving_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
  moving_forward_ = true;

  Eigen::Vector3d viewpoint_resolution = viewpoint_manager_->GetResolution();
  double add_non_keypose_node_min_dist = std::min(viewpoint_resolution.x(), viewpoint_resolution.y()) / 2;
  keypose_graph_->SetAddNonKeyposeNodeMinDist() = add_non_keypose_node_min_dist;
  // merger_graph_  最小添加  节点  距离
  merger_graph_->SetAddNonKeyposeNodeMinDist() = add_non_keypose_node_min_dist;
  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;

  last_robot_position_ = robot_position_;
}

SensorCoveragePlanner3D::SensorCoveragePlanner3D()
    : Node("multi_tare_planner_node")
    , keypose_cloud_update_(false)
    , initialized_(false)
    , lookahead_point_update_(false)
    , relocation_(false)
    , start_exploration_(false)
    , exploration_finished_(false)
    , near_home_(false)
    , at_home_(false)
    , stopped_(false)
    , test_point_update_(false)
    , viewpoint_ind_update_(false)
    , step_(false)
    , use_momentum_(false)
    , lookahead_point_in_line_of_sight_(true)
    , reset_waypoint_(false)
    , registered_cloud_count_(0)
    , keypose_count_(0)
    , direction_change_count_(0)
    , direction_no_change_count_(0)
    , momentum_activation_count_(0)
    , reset_waypoint_joystick_axis_value_(-1.0) 
{
  is_exploring_ = true;
  last_isglobal_tsp_ = false;
  is_close_to_goal_cell_ = false;
  robot_statu_ = grid_world_ns::RobotStatus::Exploring;
  std::cout << "finished constructor" << std::endl;
}

bool SensorCoveragePlanner3D::initialize() {
  ReadParameters();
  InitializeData();
  last_goal.point.x = -1;
  last_goal.point.y = -1;
  last_goal.point.z = -1;

  robot_id_ = (int)robot_name.back() - (int)('0');
  grid_world_->allocation_strategy_ = allocation_strategy_;
  keypose_graph_->SetAllowVerticalEdge(false);
  merger_graph_->SetAllowVerticalEdge(false);

  lidar_model_ns::LiDARModel::setCloudDWZResol(planning_env_->GetPlannerCloudResolution());

  execution_timer_ = this->create_wall_timer(1000ms, std::bind(&SensorCoveragePlanner3D::execute_grid_merger_graph, this)); // 要先启动tare，再启动env

  exploration_start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      sub_start_exploration_topic_, 5,
      std::bind(&SensorCoveragePlanner3D::ExplorationStartCallback, this,
                std::placeholders::_1));
  registered_scan_sub_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
          sub_registered_scan_topic_, 5,
          std::bind(&SensorCoveragePlanner3D::RegisteredScanCallback, this,
                    std::placeholders::_1));
  terrain_map_sub_ = 
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        sub_terrain_map_topic_, 5,
        std::bind(&SensorCoveragePlanner3D::TerrainMapCallback, this,
                  std::placeholders::_1));
  terrain_map_ext_sub_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
          sub_terrain_map_ext_topic_, 5,
          std::bind(&SensorCoveragePlanner3D::TerrainMapExtCallback, this,
                    std::placeholders::_1));
  state_estimation_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      sub_state_estimation_topic_, 5,
      std::bind(&SensorCoveragePlanner3D::StateEstimationCallback, this,
                std::placeholders::_1));
  coverage_boundary_sub_ =
      this->create_subscription<geometry_msgs::msg::PolygonStamped>(
          sub_coverage_boundary_topic_, 5,
          std::bind(&SensorCoveragePlanner3D::CoverageBoundaryCallback, this,
                    std::placeholders::_1));
  viewpoint_boundary_sub_ =
      this->create_subscription<geometry_msgs::msg::PolygonStamped>(
          sub_viewpoint_boundary_topic_, 5,
          std::bind(&SensorCoveragePlanner3D::ViewPointBoundaryCallback, this,
                    std::placeholders::_1));
  nogo_boundary_sub_ = 
      this->create_subscription<geometry_msgs::msg::PolygonStamped>(
          sub_nogo_boundary_topic_, 5, 
          std::bind(&SensorCoveragePlanner3D::NogoBoundaryCallback, this, 
                    std::placeholders::_1));
                    
  // 新增
  NewGridWorldCell_and_status_sub_ =
      this->create_subscription<std_msgs::msg::Int32MultiArray>(
          "/NewGridWorldCellStatus", 5,
          std::bind(&SensorCoveragePlanner3D::NewGridWorldCellStatusCallback, this,
                    std::placeholders::_1));

  // 新增
  MTSP_SubGrapher_sub_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "/MTSP_SubGrapher", 5,
          std::bind(&SensorCoveragePlanner3D::MTSP_SubGrapherCallback, this,
                    std::placeholders::_1));

  // 新增
  Subgraph_sub_ =
      this->create_subscription<tare_planner::msg::Subgraph>(
          "/Subgraph", 5,
          std::bind(&SensorCoveragePlanner3D::SubgraphCallback, this,
                    std::placeholders::_1));

  // 新增
  is_returning_home_sub_ =
      this->create_subscription<tare_planner::msg::ReturnHome>(
          "/is_returning_home", 3,
          std::bind(&SensorCoveragePlanner3D::is_returning_home_Callback, this,
                    std::placeholders::_1));

  // 新增
  Shared_Infor_sub_ =
      this->create_subscription<tare_planner::msg::SharedMergerInfor>(
          "/SharedMergerInfor", 10000,
          std::bind(&SensorCoveragePlanner3D::SharedMergerInforCallback, this,
                    std::placeholders::_1));

  // 新增
  MergerGraphEdge_sub_ =
      this->create_subscription<tare_planner::msg::MergerGraphEdge>(
          "/MergerGraphEdge", 10000,
          std::bind(&SensorCoveragePlanner3D::MergerGraphEdgeCallback, this,
                    std::placeholders::_1));

  joystick_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      sub_joystick_topic_, 5,
      std::bind(&SensorCoveragePlanner3D::JoystickCallback, this,
                std::placeholders::_1));
  reset_waypoint_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      sub_reset_waypoint_topic_, 1,
      std::bind(&SensorCoveragePlanner3D::ResetWaypointCallback, this,
                std::placeholders::_1));

  global_path_full_publisher_ = this->create_publisher<nav_msgs::msg::Path>("global_path_full", 1);
  global_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 1);
  old_global_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("old_global_path", 1);
  to_nearest_global_subspace_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("to_nearest_global_subspace_path", 1);
  local_tsp_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("local_path", 1);
  exploration_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("exploration_path", 1);
  waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(pub_waypoint_topic_, 2);
  move_base_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/"+robot_name+"/move_base_simple/goal",2); // 新增
  exploration_finish_pub_ = this->create_publisher<std_msgs::msg::Bool>(pub_exploration_finish_topic_, 2);
  runtime_breakdown_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(pub_runtime_breakdown_topic_, 2);
  runtime_pub_ = this->create_publisher<std_msgs::msg::Float32>(pub_runtime_topic_, 2);
  momentum_activation_count_pub_ = this->create_publisher<std_msgs::msg::Int32>(pub_momentum_activation_count_topic_, 2);
  // Debug
  pointcloud_manager_neighbor_cells_origin_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("pointcloud_manager_neighbor_cells_origin", 1);

  NewGridWorldCell_and_status_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/NewGridWorldCellStatus", 1); // 新增
  MTSP_SubGrapher_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/MTSP_SubGrapher", 1); // 新增
  need_farplaner_planning_pub_ = this->create_publisher<std_msgs::msg::Bool>("/"+robot_name+"/need_goal_planning", 1); // 新增
  goal_to_farplaner_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/"+robot_name+"/goal_point", 1); // 新增
  request_path_server_ = this->create_service<tare_planner::srv::RequestPath>("/"+robot_name+"/request_path_server", std::bind(&SensorCoveragePlanner3D::request_path_server, this, std::placeholders::_1, std::placeholders::_2)); // 新增
  for (int i = 1; i <= robot_num; i++)
  {
    auto request_path_client_ = this->create_client<tare_planner::srv::RequestPath>("/" + robot_public_name_ + "_" +std::to_string(i) + "/request_path_server");
    request_path_client_list_.push_back(request_path_client_);  //  地址0 ->robot1; 地址1->robot2 ;地址2->robot3;
  }

  current_subgraph_pub_ = this->create_publisher<tare_planner::msg::Subgraph>("/Subgraph", 1); // 新增
  is_returning_home_pub_ = this->create_publisher<tare_planner::msg::ReturnHome>("/is_returning_home", 1); // 新增
  SharedMergerInfor_pub_ = this->create_publisher<tare_planner::msg::SharedMergerInfor>("/SharedMergerInfor", 1); // 新增
  MergerGraphEdge_pub_ = this->create_publisher<tare_planner::msg::MergerGraphEdge>("/MergerGraphEdge", 1); // 新增
  PrintExplorationStatus("Exploration Started", false);
  return true;
}

// 新增 request_path_server 服务端回调函数
bool SensorCoveragePlanner3D::request_path_server(const std::shared_ptr<tare_planner::srv::RequestPath::Request> req, std::shared_ptr<tare_planner::srv::RequestPath::Response> res)
{
  bool is_get = false; // 从请求中获取两个目标点，在本地keyposegrapher图上查找对应点并计算路径
  is_get = keypose_graph_->GetShortestPathTwoPoint(req->start, req->goal, true, res->path_return, true, kCellSize_);// 获取与req->goal和req->start相关联的keypose图上的节点并计算路径
  return is_get;
}

void SensorCoveragePlanner3D::ExplorationStartCallback(const std_msgs::msg::Bool::ConstSharedPtr start_msg) 
{
  if (start_msg->data) 
  {
    start_exploration_ = true;
  }
}

void SensorCoveragePlanner3D::StateEstimationCallback(const nav_msgs::msg::Odometry::ConstSharedPtr state_estimation_msg) {
  robot_position_ = state_estimation_msg->pose.pose.position;
  if (std::abs(initial_position_.x()) < 0.01 && std::abs(initial_position_.y()) < 0.01 && std::abs(initial_position_.z()) < 0.01) 
  {
    initial_position_.x() = robot_position_.x;
    initial_position_.y() = robot_position_.y;
    initial_position_.z() = robot_position_.z;
  }
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geo_quat = state_estimation_msg->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w)).getRPY(roll, pitch, yaw);
  robot_yaw_ = yaw;
  if (state_estimation_msg->twist.twist.linear.x > 0.4) 
  {
    moving_forward_ = true;
  } 
  else if (state_estimation_msg->twist.twist.linear.x < -0.4) 
  {
    moving_forward_ = false;
  }
  initialized_ = true;
}

void SensorCoveragePlanner3D::RegisteredScanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg) {
  if (!initialized_) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_tmp(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*registered_scan_msg, *registered_scan_tmp);
  if (registered_scan_tmp->points.empty()) 
  {
    return;
  }
  *(registered_scan_stack_->cloud_) += *(registered_scan_tmp);
  pointcloud_downsizer_.Downsize(registered_scan_tmp, kKeyposeCloudDwzFilterLeafSize, kKeyposeCloudDwzFilterLeafSize, kKeyposeCloudDwzFilterLeafSize);
  registered_cloud_->cloud_->clear();
  pcl::copyPointCloud(*registered_scan_tmp, *(registered_cloud_->cloud_));

  planning_env_->UpdateRobotPosition(robot_position_);
  planning_env_->UpdateRegisteredCloud<pcl::PointXYZI>(registered_cloud_->cloud_);

  registered_cloud_count_ = (registered_cloud_count_ + 1) % 5;
  if (registered_cloud_count_ == 0) 
  {
    keypose_.pose.pose.position = robot_position_;
    keypose_.pose.covariance[0] = keypose_count_++;
    cur_keypose_node_ind_ = keypose_graph_->AddKeyposeNode(keypose_, *(planning_env_));
    pointcloud_downsizer_.Downsize(registered_scan_stack_->cloud_, kKeyposeCloudDwzFilterLeafSize,
                                   kKeyposeCloudDwzFilterLeafSize, kKeyposeCloudDwzFilterLeafSize);
    keypose_cloud_->cloud_->clear();
    pcl::copyPointCloud(*(registered_scan_stack_->cloud_), *(keypose_cloud_->cloud_));
    registered_scan_stack_->cloud_->clear();
    keypose_cloud_update_ = true;
  }
}

void SensorCoveragePlanner3D::TerrainMapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_msg) {
  if (kCheckTerrainCollision) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_map_tmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_msg, *terrain_map_tmp);
    terrain_collision_cloud_->cloud_->clear();
    for (auto &point : terrain_map_tmp->points) {
      if (point.intensity > kTerrainCollisionThreshold) {
        terrain_collision_cloud_->cloud_->points.push_back(point);
      }
    }
  }
}

void SensorCoveragePlanner3D::TerrainMapExtCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_ext_msg) {
  if (kUseTerrainHeight) 
  {
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_ext_msg, *(large_terrain_cloud_->cloud_));
  }
  if (kCheckTerrainCollision) {
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_ext_msg, *(large_terrain_cloud_->cloud_));
    terrain_ext_collision_cloud_->cloud_->clear();
    for (auto &point : large_terrain_cloud_->cloud_->points) {
      if (point.intensity > kTerrainCollisionThreshold) {
        terrain_ext_collision_cloud_->cloud_->points.push_back(point);
      }
    }
  }
}

void SensorCoveragePlanner3D::CoverageBoundaryCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygon_msg) {
  planning_env_->UpdateCoverageBoundary((*polygon_msg).polygon);
}

void SensorCoveragePlanner3D::ViewPointBoundaryCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygon_msg) {
  viewpoint_manager_->UpdateViewPointBoundary((*polygon_msg).polygon);
}

void SensorCoveragePlanner3D::NogoBoundaryCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygon_msg) {
  if (polygon_msg->polygon.points.empty()) 
  {
    return;
  }
  double polygon_id = polygon_msg->polygon.points[0].z;
  int polygon_point_size = polygon_msg->polygon.points.size();
  std::vector<geometry_msgs::msg::Polygon> nogo_boundary;
  geometry_msgs::msg::Polygon polygon;
  for (int i = 0; i < polygon_point_size; i++) {
    if (polygon_msg->polygon.points[i].z == polygon_id) {
      polygon.points.push_back(polygon_msg->polygon.points[i]);
    } else {
      nogo_boundary.push_back(polygon);
      polygon.points.clear();
      polygon_id = polygon_msg->polygon.points[i].z;
      polygon.points.push_back(polygon_msg->polygon.points[i]);
    }
  }
  nogo_boundary.push_back(polygon);
  viewpoint_manager_->UpdateNogoBoundary(nogo_boundary);

  geometry_msgs::msg::Point point;
  for (int i = 0; i < nogo_boundary.size(); i++) {
    for (int j = 0; j < nogo_boundary[i].points.size() - 1; j++) {
      point.x = nogo_boundary[i].points[j].x;
      point.y = nogo_boundary[i].points[j].y;
      point.z = nogo_boundary[i].points[j].z;
      nogo_boundary_marker_->marker_.points.push_back(point);
      point.x = nogo_boundary[i].points[j + 1].x;
      point.y = nogo_boundary[i].points[j + 1].y;
      point.z = nogo_boundary[i].points[j + 1].z;
      nogo_boundary_marker_->marker_.points.push_back(point);
    }
    point.x = nogo_boundary[i].points.back().x;
    point.y = nogo_boundary[i].points.back().y;
    point.z = nogo_boundary[i].points.back().z;
    nogo_boundary_marker_->marker_.points.push_back(point);
    point.x = nogo_boundary[i].points.front().x;
    point.y = nogo_boundary[i].points.front().y;
    point.z = nogo_boundary[i].points.front().z;
    nogo_boundary_marker_->marker_.points.push_back(point);
  }
  nogo_boundary_marker_->Publish();
}


// 新增回调：网格世界更新
void SensorCoveragePlanner3D::NewGridWorldCellStatusCallback(const std_msgs::msg::Int32MultiArray::ConstSharedPtr NewGridWorldCellStatus_msg)
{
  if ((int)robot_name.back() != NewGridWorldCellStatus_msg->data.front())  
  {
    if (initialized_)  
    {
      int num = NewGridWorldCellStatus_msg->data.size() - 1;  
      int grid_world_ID;
      int statu;
      
      for (int i = 1; i <= (num / 2); i++)  
      {
        grid_world_ID = NewGridWorldCellStatus_msg->data[2 * i - 1];
        statu = NewGridWorldCellStatus_msg->data[2 * i];
        
        if (Update_Grid_World_ID_and_Statu_.count(grid_world_ID)) 
        {
          if ((Update_Grid_World_ID_and_Statu_[grid_world_ID] == 2 ||
               Update_Grid_World_ID_and_Statu_[grid_world_ID] == 3) &&
              statu == 1)  
          {
            continue;
          }
          else if (statu < Update_Grid_World_ID_and_Statu_[grid_world_ID]) 
          {
            continue;
          }
          else 
          {
            Update_Grid_World_ID_and_Statu_[grid_world_ID] = statu;
          }
        }
        else 
        {
          Update_Grid_World_ID_and_Statu_.insert(std::pair<int, int>(grid_world_ID, statu));  
        }
      }
    }
  }
}

// 新增回调：接受MTSP子图
void SensorCoveragePlanner3D::MTSP_SubGrapherCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr MTSP_SubGrapher_msg)
{
  int sender_id = (int)(MTSP_SubGrapher_msg->data.front());
  int robot_ID = (int)(robot_name.back()) - (int)('0');
  
  if (robot_ID != sender_id)  // 仅处理其他机器人消息
  {
    if (initialized_)  // 已初始化状态
    {
      int subgraph_node_num = static_cast<int>(MTSP_SubGrapher_msg->data[1]); 
      int node_num = static_cast<int>(MTSP_SubGrapher_msg->data[1]);
      int pos_start_idx = 3 * node_num - 1;  
      
      geometry_msgs::msg::Point position;
      position.x = MTSP_SubGrapher_msg->data[pos_start_idx];
      position.y = MTSP_SubGrapher_msg->data[pos_start_idx + 1];
      position.z = MTSP_SubGrapher_msg->data[pos_start_idx + 2];
      
      other_robot_position_map_;
      if (other_robot_position_map_.count(sender_id))
      {
        other_robot_position_map_[sender_id] = position;
      }
      else
      {
        other_robot_position_map_.insert(std::pair<int, geometry_msgs::msg::Point>(sender_id, position));
      }
      
      std::vector<double> MTSP_SubGrapher;
      for (size_t i = 1; i < MTSP_SubGrapher_msg->data.size(); ++i)
      {
        MTSP_SubGrapher.push_back(MTSP_SubGrapher_msg->data[i]);
      }
      
      if (MTSP_subgrapher_map_.count(sender_id))
      {
        MTSP_subgrapher_map_[sender_id] = MTSP_SubGrapher;
      }
      else
      {
        MTSP_subgrapher_map_.insert(std::pair<int, std::vector<double>>(sender_id, MTSP_SubGrapher));
      }
    }
  }
}

// 新增回调：网格子图
void SensorCoveragePlanner3D::SubgraphCallback(const tare_planner::msg::Subgraph::ConstSharedPtr Subgraph_msg)
{
  // 过滤本地机器人消息
  if (Subgraph_msg->robot_id != robot_id_)
  {
    // 初始化历史位置容器（若不存在）
    if (other_robot_history_position_.count(Subgraph_msg->robot_id) == 0)
    {
      std::vector<geometry_msgs::msg::Point> temp_points;
      other_robot_history_position_.insert(std::pair<int, std::vector<geometry_msgs::msg::Point>>(Subgraph_msg->robot_id, temp_points));
    }
    
    // 记录机器人当前位置到历史轨迹
    other_robot_history_position_[Subgraph_msg->robot_id].push_back(Subgraph_msg->robot_postion);
    
    // 初始化子图节点容器（若不存在）
    if (robot_cell_subnode_.count(Subgraph_msg->robot_id) == 0)
    {
      std::map<int, tare_planner::msg::Subnode> temp_map;
      robot_cell_subnode_.insert(std::pair<int, std::map<int, tare_planner::msg::Subnode>>(Subgraph_msg->robot_id, temp_map));
    }
    
    // 更新子图节点（覆盖或新增）
    for (const auto& node : Subgraph_msg->neighbor_nodes)  // ROS 2消息字段建议使用小写蛇形命名
    {
      auto& subnode_map = robot_cell_subnode_[Subgraph_msg->robot_id];
      if (subnode_map.count(node.cell_index))
      {
        subnode_map[node.cell_index] = node;  // 更新已有节点
      }
      else
      {
        subnode_map.emplace(node.cell_index, node);  // 新增节点
      }
    }
  }
}

// 新增回调：是否返回起点
void SensorCoveragePlanner3D::is_returning_home_Callback(const tare_planner::msg::ReturnHome::ConstSharedPtr is_returning_home_msg)
{
  if (is_returning_home_list.size() >= static_cast<size_t>(is_returning_home_msg->robot_id))
  {
    is_returning_home_list[is_returning_home_msg->robot_id - 1] = is_returning_home_msg->is_returning_home; // 更新返回状态（索引从0开始，机器人ID从1开始）
  }
}

// 新增回调：共享融合信息
void SensorCoveragePlanner3D::SharedMergerInforCallback(const tare_planner::msg::SharedMergerInfor::ConstSharedPtr SharedMergerInfor_msg)
{
  if (SharedMergerInfor_msg->robot_id == robot_id_)
  {
    return;
  }
  
  // 合并新节点
  for (const auto& node : SharedMergerInfor_msg->new_node_set)
  {
    Shared_Infor_map_[SharedMergerInfor_msg->robot_id].new_node_set.push_back(node);
    if (node.is_keypose)
    {
      other_robot_position_map_[SharedMergerInfor_msg->robot_id] = node.node_position; // 若为关键点，则更新机器人位置
    }
  }
  
  // 合并新增边
  for (const auto& edge : SharedMergerInfor_msg->add_edge_set)
  {
    Shared_Infor_map_[SharedMergerInfor_msg->robot_id].add_edge_set.push_back(edge);
  }
  
  // 合并删除边
  for (const auto& edge : SharedMergerInfor_msg->delete_edge_set)
  {
    Shared_Infor_map_[SharedMergerInfor_msg->robot_id].delete_edge_set.push_back(edge);
  }
}

// 新增回调：融合图边更新
void SensorCoveragePlanner3D::MergerGraphEdgeCallback(const tare_planner::msg::MergerGraphEdge::ConstSharedPtr MergerGraphEdge_msg)
{
  if (MergerGraphEdge_msg->robot_id == robot_id_)
  {
    return; // 过滤本地机器人消息
  }
  
  // 合并新增边和删除边
  for (const auto& edge : MergerGraphEdge_msg->add_edge_set)
  {
    MergerGraphEdge_Once_sub_.add_edge_set.push_back(edge);
  }
  for (const auto& edge : MergerGraphEdge_msg->delete_edge_set)
  {
    MergerGraphEdge_Once_sub_.delete_edge_set.push_back(edge);
  }
}

void SensorCoveragePlanner3D::JoystickCallback(
    const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg) {
  if (kResetWaypointJoystickAxesID >= 0 &&
      kResetWaypointJoystickAxesID < joy_msg->axes.size()) {
    if (reset_waypoint_joystick_axis_value_ > -0.1 &&
        joy_msg->axes[kResetWaypointJoystickAxesID] < -0.1) {
      reset_waypoint_ = true;

      // Set waypoint to the current robot position to stop the robot in place
      geometry_msgs::msg::PointStamped waypoint;
      waypoint.header.frame_id = robot_name+"/"+"map";
      waypoint.header.stamp = this->now();
      waypoint.point.x = robot_position_.x;
      waypoint.point.y = robot_position_.y;
      waypoint.point.z = robot_position_.z;
      waypoint_pub_->publish(waypoint);
      std::cout << "reset waypoint" << std::endl;
    }
    reset_waypoint_joystick_axis_value_ =
        joy_msg->axes[kResetWaypointJoystickAxesID];
  }
}

void SensorCoveragePlanner3D::ResetWaypointCallback(
    const std_msgs::msg::Empty::ConstSharedPtr empty_msg) {
  reset_waypoint_ = true;

  // Set waypoint to the current robot position to stop the robot in place
  geometry_msgs::msg::PointStamped waypoint;
  waypoint.header.frame_id = robot_name+"/"+"map";
  waypoint.header.stamp = this->now();
  waypoint.point.x = robot_position_.x;
  waypoint.point.y = robot_position_.y;
  waypoint.point.z = robot_position_.z;
  waypoint_pub_->publish(waypoint);
  std::cout << "reset waypoint" << std::endl;
}

void SensorCoveragePlanner3D::SendInitialWaypoint() {
  // send waypoint ahead
  double lx = 12.0;
  double ly = 0.0;
  double dx = cos(robot_yaw_) * lx - sin(robot_yaw_) * ly;
  double dy = sin(robot_yaw_) * lx + cos(robot_yaw_) * ly;

  geometry_msgs::msg::PointStamped waypoint;
  waypoint.header.frame_id = robot_name + "/" + "map";  //改变坐标系 为  robot0/map
  waypoint.header.stamp = this->now();
  waypoint.point.x = robot_position_.x + dx;
  waypoint.point.y = robot_position_.y + dy;
  waypoint.point.z = robot_position_.z;
  waypoint_pub_->publish(waypoint);
}

void SensorCoveragePlanner3D::UpdateKeyposeGraph() {
  misc_utils_ns::Timer update_keypose_graph_timer("update keypose graph");
  update_keypose_graph_timer.Start();
  keypose_graph_->GetMarker(keypose_graph_node_marker_->marker_, keypose_graph_edge_marker_->marker_);
  // keypose_graph_node_marker_->Publish();
  keypose_graph_edge_marker_->Publish();
  keypose_graph_vis_cloud_->cloud_->clear();

  // 检测碰撞，更新边关系（局部规划框范围内）
  keypose_graph_->CheckLocalCollision(robot_position_, viewpoint_manager_);
  keypose_graph_->CheckConnectivity(robot_position_);
  keypose_graph_->GetVisualizationCloud(keypose_graph_vis_cloud_->cloud_);
  keypose_graph_vis_cloud_->Publish();
  update_keypose_graph_timer.Stop(true);
}

// 更新 拼接图
void SensorCoveragePlanner3D::UpdateMergerGraph()
{
  misc_utils_ns::Timer update_merger_graph_timer("update merger graph");
  update_merger_graph_timer.Start();

  // 定义 单次 维护 merger_graph 添加的边和删除的边
  std::vector<std::pair<int, int>> delete_edge;
  std::vector<std::pair<int, int>> add_edge;

  // 维护 添加点在grid_graph中，使用它的网格信息，加点时查找同一网格的点能否融合
  // 融合条件为距离在加点距离的1/2，做点融合
  grid_world_->AddNode2MergerGraph(merger_graph_, Shared_Infor_map_);
  // 添加单机器人的边
  merger_graph_->ProcessOneceDataAddEdge(Shared_Infor_map_);
  Shared_Infor_map_.clear();

  // 获取拼接图需要更改的边关系 MergerGraphEdge_Once_sub_
  merger_graph_->UpdataOnceMergerGraphEdgeFromOthers(MergerGraphEdge_Once_sub_);

  MergerGraphEdge_Once_sub_.add_edge_set.clear();
  MergerGraphEdge_Once_sub_.delete_edge_set.clear();

  // 在邻接网格中添加不同机器人之间的边
  // 传入merger_garph和viewpoint_manager_做碰撞检测
  grid_world_->AddDiffRobotEdge2MergerGraphOnNeighborCell(merger_graph_, viewpoint_manager_, add_edge);

  // 添加与当前机器人在范围内的其他机器人节点的连线
  merger_graph_->AddEdgeLocalRobotWithOtherRobotNodesInRange(robot_position_, merger_graph_->SetAddEdgeConnectDistThr(), robot_id_, cur_keypose_node_ind_, viewpoint_manager_, add_edge);

  // 可视化处理
  merger_graph_->GetMarker(merger_graph_node_marker_->marker_, merger_graph_edge_marker_->marker_);
  merger_graph_edge_marker_->Publish(); 
  merger_graph_vis_cloud_->cloud_->clear();

  // 检测碰撞，更新边关系（局部规划框范围内）
  merger_graph_->CheckLocalCollisionMergerGraph(robot_position_, viewpoint_manager_, delete_edge);

  // 准备发布的边更新消息
  tare_planner::msg::MergerGraphEdge shared_merger_graph_edge;  // ROS2消息位于msg子命名空间
  merger_graph_->UpdataOnceMergerGraphEdge(shared_merger_graph_edge, add_edge, delete_edge);
  shared_merger_graph_edge.robot_id = robot_id_;
  
  MergerGraphEdge_pub_->publish(shared_merger_graph_edge);

  // 维护ikdtree
  merger_graph_->CheckConnectivityIkdtree(robot_position_);  // 检测连通性
  merger_graph_->GetVisualizationCloud(merger_graph_vis_cloud_->cloud_);
  merger_graph_vis_cloud_->Publish(); 

  update_merger_graph_timer.Stop(true);
}

int SensorCoveragePlanner3D::UpdateViewPoints() {
  misc_utils_ns::Timer collision_cloud_timer("update collision cloud");
  collision_cloud_timer.Start();
  collision_cloud_->cloud_ = planning_env_->GetCollisionCloud();
  collision_cloud_timer.Stop(false);

  misc_utils_ns::Timer viewpoint_manager_update_timer("update viewpoint manager");
  viewpoint_manager_update_timer.Start();
  if (kUseTerrainHeight) {
    viewpoint_manager_->SetViewPointHeightWithTerrain(
        large_terrain_cloud_->cloud_);
  }
  if (kCheckTerrainCollision) {
    *(collision_cloud_->cloud_) += *(terrain_collision_cloud_->cloud_);
    *(collision_cloud_->cloud_) += *(terrain_ext_collision_cloud_->cloud_);
  }
  // 新增
  viewpoint_manager_->is_exploring_ = is_exploring_;
  viewpoint_manager_->robot_statu_ = robot_statu_;
  viewpoint_manager_->last_isglobal_tsp_ = last_isglobal_tsp_;
  int cell_index = grid_world_->GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
  viewpoint_manager_->robot_cell_statu = grid_world_->GetCellStatus_world(cell_index);

  viewpoint_manager_->CheckViewPointCollision(collision_cloud_->cloud_);
  viewpoint_manager_->CheckViewPointLineOfSight();
  viewpoint_manager_->CheckViewPointConnectivity();
  int viewpoint_candidate_count = viewpoint_manager_->GetViewPointCandidate();

  UpdateVisitedPositions();
  viewpoint_manager_->UpdateViewPointVisited(visited_positions_);
  viewpoint_manager_->UpdateViewPointVisited(grid_world_);

  // For visualization
  collision_cloud_->Publish();
  // collision_grid_cloud_->Publish();
  viewpoint_manager_->GetCollisionViewPointVisCloud(
      viewpoint_in_collision_cloud_->cloud_);
  viewpoint_in_collision_cloud_->Publish();

  viewpoint_manager_update_timer.Stop(false);
  return viewpoint_candidate_count;
}

void SensorCoveragePlanner3D::UpdateViewPointCoverage() {
  // Update viewpoint coverage
  misc_utils_ns::Timer update_coverage_timer("update viewpoint coverage");
  update_coverage_timer.Start();
  viewpoint_manager_->UpdateViewPointCoverage<PlannerCloudPointType>(planning_env_->GetDiffCloud());
  viewpoint_manager_->UpdateRolledOverViewPointCoverage<PlannerCloudPointType>(planning_env_->GetStackedCloud());
  // Update robot coverage
  robot_viewpoint_.ResetCoverage();
  geometry_msgs::msg::Pose robot_pose;
  robot_pose.position = robot_position_;
  robot_viewpoint_.setPose(robot_pose);
  UpdateRobotViewPointCoverage();
  update_coverage_timer.Stop(true);
}

void SensorCoveragePlanner3D::UpdateRobotViewPointCoverage() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = planning_env_->GetCollisionCloud();
  for (const auto &point : cloud->points) {
    if (viewpoint_manager_->InFOVAndRange(
            Eigen::Vector3d(point.x, point.y, point.z),
            Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z))) 
    {
      robot_viewpoint_.UpdateCoverage<pcl::PointXYZI>(point);
    }
  }
}

void SensorCoveragePlanner3D::UpdateCoveredAreas(int &uncovered_point_num, int &uncovered_frontier_point_num) {
  // Update covered area
  misc_utils_ns::Timer update_coverage_area_timer("update covered area");
  update_coverage_area_timer.Start();
  planning_env_->UpdateCoveredArea(robot_viewpoint_, viewpoint_manager_);

  update_coverage_area_timer.Stop(true);
  misc_utils_ns::Timer get_uncovered_area_timer("get uncovered area");
  get_uncovered_area_timer.Start();
  planning_env_->GetUncoveredArea(viewpoint_manager_, uncovered_point_num, uncovered_frontier_point_num);

  get_uncovered_area_timer.Stop(true);
  planning_env_->PublishUncoveredCloud();
  planning_env_->PublishUncoveredFrontierCloud();
}

void SensorCoveragePlanner3D::UpdateVisitedPositions() {
  Eigen::Vector3d robot_current_position(robot_position_.x, robot_position_.y, robot_position_.z);
  bool existing = false;
  for (int i = 0; i < visited_positions_.size(); i++) {
    // TODO: parameterize this
    if ((robot_current_position - visited_positions_[i]).norm() < 1) {
      existing = true;
      break;
    }
  }
  if (!existing) {
    visited_positions_.push_back(robot_current_position);

    // 新增
    exploration_path_ns::Node node;
    node.position_.x() = robot_position_.x;
    node.position_.y() = robot_position_.y;
    node.position_.z() = robot_position_.z;
    // 计算所在 index
    node.global_subspace_index_ = grid_world_->GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
    node.type_ = exploration_path_ns::NodeType::ROBOT;
    historical_path_.nodes_.push_back(node);
  }
}

void SensorCoveragePlanner3D::UpdateGlobalRepresentation() {
  // 新增
  misc_utils_ns::Timer update_Global_ENV_timer("update Global ENV");
  update_Global_ENV_timer.Start();

  local_coverage_planner_->SetRobotPosition(
      Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z));
  bool viewpoint_rollover = viewpoint_manager_->UpdateRobotPosition(
      Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z));
  if (!grid_world_->Initialized() || viewpoint_rollover) {
    grid_world_->UpdateNeighborCells(robot_position_);
  }

  planning_env_->UpdateRobotPosition(robot_position_);
  planning_env_->GetVisualizationPointCloud(point_cloud_manager_neighbor_cloud_->cloud_);
  point_cloud_manager_neighbor_cloud_->Publish();

  // DEBUG
  Eigen::Vector3d pointcloud_manager_neighbor_cells_origin =
      planning_env_->GetPointCloudManagerNeighborCellsOrigin();
  geometry_msgs::msg::PointStamped pointcloud_manager_neighbor_cells_origin_point;
  pointcloud_manager_neighbor_cells_origin_point.header.frame_id = robot_name + "/" + "map";
  pointcloud_manager_neighbor_cells_origin_point.header.stamp = this->now();
  pointcloud_manager_neighbor_cells_origin_point.point.x = pointcloud_manager_neighbor_cells_origin.x();
  pointcloud_manager_neighbor_cells_origin_point.point.y = pointcloud_manager_neighbor_cells_origin.y();
  pointcloud_manager_neighbor_cells_origin_point.point.z = pointcloud_manager_neighbor_cells_origin.z();
  pointcloud_manager_neighbor_cells_origin_pub_->publish(pointcloud_manager_neighbor_cells_origin_point);

  if (exploration_finished_ && kNoExplorationReturnHome) {
    planning_env_->SetUseFrontier(false);
  }
  planning_env_->UpdateKeyposeCloud<PlannerCloudPointType>(keypose_cloud_->cloud_);

  int closest_node_ind = keypose_graph_->GetClosestNodeInd(robot_position_);
  geometry_msgs::msg::Point closest_node_position = keypose_graph_->GetClosestNodePosition(robot_position_);
  grid_world_->SetCurKeyposeGraphNodeInd(closest_node_ind);
  grid_world_->SetCurKeyposeGraphNodePosition(closest_node_position);

  grid_world_->UpdateRobotPosition(robot_position_);
  if (!grid_world_->HomeSet()) {
    grid_world_->SetHomePosition(initial_position_);
  }
   update_Global_ENV_timer.Stop(true);
}

// 新增更新全局状态
void SensorCoveragePlanner3D::GlobalStatuUpdate()
{
  misc_utils_ns::Timer global_tsp_timer("Global StatuUpdate");
  global_tsp_timer.Start();
  
  // 更新来自其他机器人的网格世界状态
  grid_world_->UpdateGridWorldCellFromOtherRobots(Update_Grid_World_ID_and_Statu_);
  
  // 网格地图状态更新
  grid_world_->UpdateCellStatus_(viewpoint_manager_);
  
  // 获取需要发送的邻接网格状态更新编码
  std::vector<int> UpdateNeighbor_GridWorldCellStatus_code_;
  grid_world_->GetUpdateNeighbor_NewGridWorldCellStatus_(UpdateNeighbor_GridWorldCellStatus_code_);
  std_msgs::msg::Int32MultiArray NewGridWorldCellStatus_msg;  
  NewGridWorldCellStatus_msg.data.clear();
  NewGridWorldCellStatus_msg.data.push_back(robot_id_); 
  
  // 填充网格状态数据
  for (int code : UpdateNeighbor_GridWorldCellStatus_code_)
  {
    NewGridWorldCellStatus_msg.data.push_back(code);
  }
  
  // 发布网格状态更新
  NewGridWorldCell_and_status_pub_->publish(NewGridWorldCellStatus_msg);

  // 更新关键位姿图节点
  grid_world_->UpdateCellKeyposeGraphNodes(keypose_graph_);
  
  // 在网格间添加路径
  grid_world_->AddPathsInBetweenCells(viewpoint_manager_, keypose_graph_);

  // 构建并发布共享融合信息
  tare_planner::msg::SharedMergerInfor Shared_Infor(keypose_graph_->GetSharedInfor());  // ROS2消息命名空间
  Shared_Infor.robot_id = robot_id_;
  SharedMergerInfor_pub_->publish(Shared_Infor);  // ROS2发布方式
  
  // 将本地共享信息添加到全局处理映射
  AddLocalSharedMergerInfor(Shared_Infor);
  
  // 清除本次更新的稀疏图信息
  keypose_graph_->ClearSharedInfor();

  // 更新候选视点单元格状态
  viewpoint_manager_->UpdateCandidateViewPointCellStatus(grid_world_);

  // 记录运行时间
  global_tsp_timer.Stop(true);
  global_planning_runtime_ = global_tsp_timer.GetDuration("ms");
}

// 修改 全局 规划 使用 merger graph 代替
void SensorCoveragePlanner3D::GlobalPlanning_grid_merger_graph(std::vector<int>& global_cell_tsp_order,
                                                               exploration_path_ns::ExplorationPath& global_path,
                                                               bool& is_global_tsp)
{
  misc_utils_ns::Timer global_tsp_timer("Global planning");
  global_tsp_timer.Start();
  int robot_ID = static_cast<int>(robot_name.back() - '0');

  // 使用 merger graph 解决全局规划获得全局路径，取代 keypose_graph
  global_path = grid_world_->SolveGlobalMdvrp_merger_graph(
      viewpoint_manager_,          // 局部规划器
      global_cell_tsp_order,           // 全局路径中的目标网格编号
      is_global_tsp,                   // 是否做全局分配规划
      request_path_client_list_,       // 请求路径服务端（ROS2服务客户端列表）
      keypose_graph_,              // 关键位姿图
      merger_graph_,               // 全局拼接图
      other_robot_position_map_);      // 其他机器人位置映射

  // 记录运行时间
  global_tsp_timer.Stop(true);
  global_planning_runtime_ = global_tsp_timer.GetDuration("ms"); 
}

void SensorCoveragePlanner3D::PublishGlobalPlanningVisualization(const exploration_path_ns::ExplorationPath &global_path,
                                                                 const exploration_path_ns::ExplorationPath &local_path) 
{
  nav_msgs::msg::Path global_path_full = global_path.GetPath();
  global_path_full.header.frame_id = robot_name + "/" + "map";
  global_path_full.header.stamp = this->now();
  global_path_full_publisher_->publish(global_path_full);

  int start_index = 0;
  for (int i = 0; i < global_path.nodes_.size(); i++) {
    if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
        global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ||
        !viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_[i].position_)) 
    {
      break;
    }
    start_index = i;
  }

  int end_index = global_path.nodes_.size() - 1;
  for (int i = global_path.nodes_.size() - 1; i >= 0; i--) {
    if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
        global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ||
        !viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_[i].position_)) 
    {
      break;
    }
    end_index = i;
  }

  nav_msgs::msg::Path global_path_trim;
  if (local_path.nodes_.size() >= 2) {
    geometry_msgs::msg::PoseStamped first_pose;
    first_pose.pose.position.x = local_path.nodes_.front().position_.x();
    first_pose.pose.position.y = local_path.nodes_.front().position_.y();
    first_pose.pose.position.z = local_path.nodes_.front().position_.z();
    global_path_trim.poses.push_back(first_pose);
  }

  for (int i = start_index; i <= end_index; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = global_path.nodes_[i].position_.x();
    pose.pose.position.y = global_path.nodes_[i].position_.y();
    pose.pose.position.z = global_path.nodes_[i].position_.z();
    global_path_trim.poses.push_back(pose);
  }
  if (local_path.nodes_.size() >= 2) {
    geometry_msgs::msg::PoseStamped last_pose;
    last_pose.pose.position.x = local_path.nodes_.back().position_.x();
    last_pose.pose.position.y = local_path.nodes_.back().position_.y();
    last_pose.pose.position.z = local_path.nodes_.back().position_.z();
    global_path_trim.poses.push_back(last_pose);
  }
  global_path_trim.header.frame_id = robot_name + "/" + "map";
  global_path_trim.header.stamp = this->now();
  global_path_publisher_->publish(global_path_trim);

  grid_world_->GetVisualizationCloud(grid_world_vis_cloud_->cloud_);
  grid_world_vis_cloud_->Publish();
  grid_world_->GetMarker(grid_world_marker_->marker_);
  grid_world_marker_->Publish();
  nav_msgs::msg::Path full_path = exploration_path_.GetPath();
  full_path.header.frame_id = robot_name + "/" + "map";
  full_path.header.stamp = this->now();
  exploration_path_.GetVisualizationCloud(exploration_path_cloud_->cloud_);
  exploration_path_cloud_->Publish();
}

// 优化的局部规划
void SensorCoveragePlanner3D::LocalPlanning_opt(int uncovered_point_num, int uncovered_frontier_point_num,
                                                const std::vector<exploration_path_ns::ExplorationPath>& near_localcoverage_subgrid_paths,
                                                exploration_path_ns::ExplorationPath& local_path)
{
  misc_utils_ns::Timer local_tsp_timer("Local planning");
  local_tsp_timer.Start();
  
  if (lookahead_point_update_)
  {
    local_coverage_planner_->SetLookAheadPoint(lookahead_point_);
  }
  
  local_coverage_planner_->SetLocalPlanningType(is_exploring_);

  // 求解优化的局部覆盖问题
  local_path = local_coverage_planner_->SolveLocalCoverageProblem_opt(near_localcoverage_subgrid_paths, uncovered_point_num, uncovered_frontier_point_num);

  RCLCPP_INFO(this->get_logger(), "local_path.nodes_.size()   =   %zu", local_path.nodes_.size());
  
  local_tsp_timer.Stop(true);
}

// 添加获取靠近局部规划框的探索子网格到机器人的路径
void SensorCoveragePlanner3D::Get_subgrid_paths(std::vector<exploration_path_ns::ExplorationPath>& near_localcoverage_subgrid_paths,
                                                std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  grid_world_->Get_subgrid_paths(near_localcoverage_subgrid_paths, keypose_graph); // 调用网格世界的方法获取子网格路径
}

void SensorCoveragePlanner3D::PublishLocalPlanningVisualization(const exploration_path_ns::ExplorationPath &local_path) 
{
  viewpoint_manager_->GetVisualizationCloud(viewpoint_vis_cloud_->cloud_);
  viewpoint_vis_cloud_->Publish();
  lookahead_point_cloud_->Publish();
  nav_msgs::msg::Path local_tsp_path = local_path.GetPath();
  local_tsp_path.header.frame_id = robot_name + "/" + "map";
  local_tsp_path.header.stamp = this->now();
  local_tsp_path_publisher_->publish(local_tsp_path);
  local_coverage_planner_->GetSelectedViewPointVisCloud(selected_viewpoint_vis_cloud_->cloud_);
  selected_viewpoint_vis_cloud_->Publish();
}

exploration_path_ns::ExplorationPath SensorCoveragePlanner3D::ConcatenateGlobalLocalPath(const exploration_path_ns::ExplorationPath &global_path,
                                                                                         const exploration_path_ns::ExplorationPath &local_path) 
{
  exploration_path_ns::ExplorationPath full_path;
  if (exploration_finished_ && near_home_ && kRushHome) 
  {
    exploration_path_ns::Node node;
    node.position_.x() = robot_position_.x;
    node.position_.y() = robot_position_.y;
    node.position_.z() = robot_position_.z;
    node.type_ = exploration_path_ns::NodeType::ROBOT;
    full_path.nodes_.push_back(node);
    node.position_ = initial_position_; // 回到初始位置
    node.type_ = exploration_path_ns::NodeType::HOME;
    full_path.nodes_.push_back(node);
    return full_path;
  }
  double global_path_length = global_path.GetLength();
  double local_path_length = local_path.GetLength();
  if (global_path_length < 3 && local_path_length < 5) 
  {
    return full_path;
  } 
  else 
  {
    full_path = local_path;
    if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_END &&
        local_path.nodes_.back().type_ == exploration_path_ns::NodeType::LOCAL_PATH_START) 
    {
      full_path.Reverse();
    } 
    else if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_START &&
             local_path.nodes_.back() == local_path.nodes_.front()) 
    {
      full_path.nodes_.back().type_ = exploration_path_ns::NodeType::LOCAL_PATH_END;
    } 
    else if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_END &&
             local_path.nodes_.back() == local_path.nodes_.front()) 
    {
      full_path.nodes_.front().type_ = exploration_path_ns::NodeType::LOCAL_PATH_START;
    }
  }
  return full_path;
}

// 新增更改：对局部路径进行排序处理
exploration_path_ns::ExplorationPath SensorCoveragePlanner3D::SortLocalPath(const exploration_path_ns::ExplorationPath& local_path)
{
  exploration_path_ns::ExplorationPath full_path;  // 实际用于探索的发布路径
  double local_path_length = local_path.GetLength();
  
  if (local_path_length < 0.5)
  {
    return full_path;  // 路径过短时，返回空路径
  }
  
  full_path = local_path;  // 以本地路径为基础
  
  // 调整路径节点类型顺序，确保第一个为开始，最后一个为结束
  if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_END &&
      local_path.nodes_.back().type_ == exploration_path_ns::NodeType::LOCAL_PATH_START)
  {
    full_path.Reverse();  // 路径颠倒
  }
  else if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_START &&
           local_path.nodes_.back() == local_path.nodes_.front())
  {
    full_path.nodes_.back().type_ = exploration_path_ns::NodeType::LOCAL_PATH_END;  // 最后一个设置为结束
  }
  else if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_END &&
           local_path.nodes_.back() == local_path.nodes_.front())
  {
    full_path.nodes_.front().type_ = exploration_path_ns::NodeType::LOCAL_PATH_START;  // 第一个设置为开始
  }
  return full_path;
}

// 新增修改：判断是否选择局部路径作为目标点
bool SensorCoveragePlanner3D::Is_Global_Planner(const exploration_path_ns::ExplorationPath& local_path)
{
  // 机器人当前位置（Eigen格式转Point格式）
  Eigen::Vector3d robot_position(robot_position_.x, robot_position_.y, robot_position_.z);
  geometry_msgs::msg::Point robot_position_Point;  // ROS2使用msg子命名空间
  robot_position_Point.x = robot_position[0];
  robot_position_Point.y = robot_position[1];
  robot_position_Point.z = robot_position[2];
  
  std::map<int, geometry_msgs::msg::Point> other_robot_position_map = other_robot_position_map_;  // ROS2消息类型
  bool local_path_too_short = true;  // 标记局部路径是否过短
  for (const auto& node : local_path.nodes_)  // 使用范围for循环更简洁
  {
    double dist_to_robot = (robot_position - node.position_).norm();  // 计算机器人到路径点的距离
    if (dist_to_robot > kLookAheadDistance / 5)
    {
      local_path_too_short = false;
      break;
    }
  }
  
  // 统计局部视点数量
  int view_num = 0;
  for (const auto& node : local_path.nodes_)
  {
    if (node.type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT)
    {
      view_num++;
    }
  }
  
  // 判断是否需要使用全局规划
  if (local_path.GetNodeNum() < 5 || local_path_too_short || view_num == 0)
  {
    return true;  // 需要使用全局路径中的点做目标点
  }
  return false;  // 局部规划可提供有效目标点，无需全局规划
}

bool SensorCoveragePlanner3D::GetLookAheadPoint(const exploration_path_ns::ExplorationPath &local_path,
                                                const exploration_path_ns::ExplorationPath &global_path,
                                                Eigen::Vector3d &lookahead_point) 
{
  Eigen::Vector3d robot_position(robot_position_.x, robot_position_.y, robot_position_.z);
  // 新增
  geometry_msgs::msg::Point robot_position_Point;  //机器人当前位置  Point  格式
  robot_position_Point.x = robot_position[0];
  robot_position_Point.y = robot_position[1];
  robot_position_Point.z = robot_position[2];
  std::map<int, geometry_msgs::msg::Point> other_robot_position_map = other_robot_position_map_;

  // 新增
  int i_begin = 1;
  int i_end = global_path.nodes_.size() - 2;

  double dist_from_start = 0.0;
  for (int i = 1; i < global_path.nodes_.size(); i++) {
    dist_from_start += (global_path.nodes_[i - 1].position_ - global_path.nodes_[i].position_).norm();
    if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT) 
    {
      i_begin = i; // 新增
      break;
    }
  }

  double dist_from_end = 0.0;
  for (int i = global_path.nodes_.size() - 2; i > 0; i--) {
    dist_from_end += (global_path.nodes_[i + 1].position_ - global_path.nodes_[i].position_).norm();
    if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT) 
    {
      i_end = i;
      break;
    }
  }

  bool local_path_too_short = true;
  for (int i = 0; i < local_path.nodes_.size(); i++) {
    double dist_to_robot = (robot_position - local_path.nodes_[i].position_).norm();
    if (dist_to_robot > kLookAheadDistance / 5) 
    {
      local_path_too_short = false;
      break;
    }
  }
  if (local_path.GetNodeNum() < 1 || local_path_too_short) {
    if (dist_from_start < dist_from_end) {
      double dist_from_robot = 0.0;
      for (int i = 1; i < global_path.nodes_.size(); i++) {
        dist_from_robot += (global_path.nodes_[i - 1].position_ - global_path.nodes_[i].position_).norm();
        if (dist_from_robot > kLookAheadDistance / 2) {
          lookahead_point = global_path.nodes_[i].position_;
          break;
        }
      }
    } 
    else {
      double dist_from_robot = 0.0;
      for (int i = global_path.nodes_.size() - 2; i > 0; i--) {
        dist_from_robot += (global_path.nodes_[i + 1].position_ - global_path.nodes_[i].position_).norm();
        if (dist_from_robot > kLookAheadDistance / 2) {
          lookahead_point = global_path.nodes_[i].position_;
          break;
        }
      }
    }
    return false;
  }

  bool has_lookahead = false;
  bool dir = true;
  int robot_i = 0;
  int lookahead_i = 0;
  for (int i = 0; i < local_path.nodes_.size(); i++) {
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::ROBOT) {
      robot_i = i;
    }
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOOKAHEAD_POINT) {
      has_lookahead = true;
      lookahead_i = i;
    }
  }

  if (reset_waypoint_) {
    has_lookahead = false;
  }

  int forward_viewpoint_count = 0;
  int backward_viewpoint_count = 0;

  bool local_loop = false;
  if (local_path.nodes_.front() == local_path.nodes_.back() &&
      local_path.nodes_.front().type_ == exploration_path_ns::NodeType::ROBOT) {
    local_loop = true;
  }

  if (local_loop) {
    robot_i = 0;
  }
  for (int i = robot_i + 1; i < local_path.GetNodeNum(); i++) {
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT) {
      forward_viewpoint_count++;
    }
  }
  if (local_loop) {
    robot_i = local_path.nodes_.size() - 1;
  }
  for (int i = robot_i - 1; i >= 0; i--) {
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT) {
      backward_viewpoint_count++;
    }
  }

  Eigen::Vector3d forward_lookahead_point = robot_position;
  Eigen::Vector3d backward_lookahead_point = robot_position;

  bool has_forward = false;
  bool has_backward = false;

  if (local_loop) 
  {
    robot_i = 0;
  }
  bool forward_lookahead_point_in_los = true;
  bool backward_lookahead_point_in_los = true;
  double length_from_robot = 0.0;
  for (int i = robot_i + 1; i < local_path.GetNodeNum(); i++) {
    length_from_robot += (local_path.nodes_[i].position_ - local_path.nodes_[i - 1].position_).norm();
    double dist_to_robot = (local_path.nodes_[i].position_ - robot_position).norm();
    bool in_line_of_sight = true;
    if (i < local_path.GetNodeNum() - 1) 
    {
      in_line_of_sight = viewpoint_manager_->InCurrentFrameLineOfSight(local_path.nodes_[i + 1].position_);
    }
    if ((length_from_robot > kLookAheadDistance || (kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_START ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_END ||
         i == local_path.GetNodeNum() - 1))

    {
      if (kUseLineOfSightLookAheadPoint && !in_line_of_sight) {
        forward_lookahead_point_in_los = false;
      }
      forward_lookahead_point = local_path.nodes_[i].position_;
      has_forward = true;
      break;
    }
  }
  if (local_loop) {
    robot_i = local_path.nodes_.size() - 1;
  }
  length_from_robot = 0.0;
  for (int i = robot_i - 1; i >= 0; i--) {
    length_from_robot += (local_path.nodes_[i].position_ - local_path.nodes_[i + 1].position_).norm();
    double dist_to_robot = (local_path.nodes_[i].position_ - robot_position).norm();
    bool in_line_of_sight = true;
    if (i > 0) 
    {
      in_line_of_sight = viewpoint_manager_->InCurrentFrameLineOfSight(local_path.nodes_[i - 1].position_);
    }
    if ((length_from_robot > kLookAheadDistance || (kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_START ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_END || i == 0))

    {
      if (kUseLineOfSightLookAheadPoint && !in_line_of_sight) 
      {
        backward_lookahead_point_in_los = false;
      }
      backward_lookahead_point = local_path.nodes_[i].position_;
      has_backward = true;
      break;
    }
  }

  if (forward_viewpoint_count > 0 && !has_forward) {
    std::cout << "forward viewpoint count > 0 but does not have forward "
                 "lookahead point"
              << std::endl;
    exit(1);
  }
  if (backward_viewpoint_count > 0 && !has_backward) {
    std::cout << "backward viewpoint count > 0 but does not have backward "
                 "lookahead point"
              << std::endl;
    exit(1);
  }

  double dx = lookahead_point_direction_.x();
  double dy = lookahead_point_direction_.y();

  double forward_angle_score = -2;
  double backward_angle_score = -2;
  double lookahead_angle_score = -2;

  double dist_robot_to_lookahead = 0.0;
  //添加影响因子      与其他机器人距离之间的影响因子
  int other_robot_num = other_robot_position_map_.size();

  if (has_forward) {
    Eigen::Vector3d forward_diff = forward_lookahead_point - robot_position;
    forward_diff.z() = 0.0;
    forward_diff = forward_diff.normalized();
    forward_angle_score = dx * forward_diff.x() + dy * forward_diff.y();
    //增加      候选点与其他机器人位置 的 距离   影响因子     得归一化   排斥其他机器人位置
    double distance_score = 0;
    geometry_msgs::msg::Point forward_lookahead_point_;  //前向点  Point  格式
    forward_lookahead_point_.x = forward_lookahead_point[0];
    forward_lookahead_point_.y = forward_lookahead_point[1];
    forward_lookahead_point_.z = forward_lookahead_point[2];
    for (auto& itr : other_robot_position_map_)
    {
      double distance =
          misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(forward_lookahead_point_, itr.second);
      if (distance < kLocal_planning_range)  //其他机器人在范围内    总分是越大越好，距离也是越远越好，排斥
      {
        double score = (distance / kLocal_planning_range) / other_robot_num;  //越远越大
        distance_score += score;
      }
      else
      {
        double score = 1 / other_robot_num;  //越远越大
        distance_score += score;
      }
    }
    forward_angle_score += distance_score;  //总体的影响是角度平滑影响加上  探索方向影响，选择距离其他机器人越远的位置
  }
  if (has_backward) {
    Eigen::Vector3d backward_diff = backward_lookahead_point - robot_position;
    backward_diff.z() = 0.0;
    backward_diff = backward_diff.normalized();
    backward_angle_score = dx * backward_diff.x() + dy * backward_diff.y();
    // 新增
    double distance_score = 0;
    geometry_msgs::msg::Point backward_lookahead_point_;  //后向点  Point  格式
    backward_lookahead_point_.x = backward_lookahead_point[0];
    backward_lookahead_point_.y = backward_lookahead_point[1];
    backward_lookahead_point_.z = backward_lookahead_point[2];
    for (auto& itr : other_robot_position_map_)
    {
      robot_position_Point;
      double distance = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
          backward_lookahead_point_, itr.second);
      if (distance < kLocal_planning_range)  //其他机器人在范围内    总分是越大越好，距离也是越远越好，排斥
      {
        double score = (distance / kLocal_planning_range) / other_robot_num;  //越远越大
        distance_score += score;
      }
      else
      {
        double score = 1 / other_robot_num;  //越远越大
        distance_score += score;
      }
    }
    backward_angle_score += distance_score;
  }
  if (has_lookahead) {
    Eigen::Vector3d prev_lookahead_point = local_path.nodes_[lookahead_i].position_;
    dist_robot_to_lookahead = (robot_position - prev_lookahead_point).norm();
    Eigen::Vector3d diff = prev_lookahead_point - robot_position;
    diff.z() = 0.0;
    diff = diff.normalized();
    lookahead_angle_score = dx * diff.x() + dy * diff.y();

   // 新增
    double distance_score = 0;
    geometry_msgs::msg::Point prev_lookahead_point_;  //前视点     Point  格式
    prev_lookahead_point_.x = prev_lookahead_point[0];
    prev_lookahead_point_.y = prev_lookahead_point[1];
    prev_lookahead_point_.z = prev_lookahead_point[2];
    for (auto& itr : other_robot_position_map_)
    {
      robot_position_Point;
      double distance =
          misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(prev_lookahead_point_, itr.second);
      if (distance < kLocal_planning_range)  //其他机器人在范围内    总分是越大越好，距离也是越远越好，排斥
      {
        double score = (distance / kLocal_planning_range) / other_robot_num;  //越远越大
        distance_score += score;
      }
      else
      {
        double score = 1 / other_robot_num;  //越远越大
        distance_score += score;
      }
    }
    lookahead_angle_score += distance_score;
  }

  lookahead_point_cloud_->cloud_->clear();

  if (forward_viewpoint_count == 0 && backward_viewpoint_count == 0) {
    relocation_ = true;
  } else {
    relocation_ = false;
  }
  if (relocation_) {
    if (use_momentum_ && kUseMomentum) {
      if (forward_angle_score > backward_angle_score) {
        lookahead_point = forward_lookahead_point;
      } else {
        lookahead_point = backward_lookahead_point;
      }
    } else {
      // follow the shorter distance one
      if (dist_from_start < dist_from_end && local_path.nodes_.front().type_ != exploration_path_ns::NodeType::ROBOT) {
        lookahead_point = backward_lookahead_point;
      } else if (dist_from_end < dist_from_start && local_path.nodes_.back().type_ != exploration_path_ns::NodeType::ROBOT) {
        lookahead_point = forward_lookahead_point;
      } else {
        lookahead_point = forward_angle_score > backward_angle_score ? forward_lookahead_point : backward_lookahead_point;
      }
    }
  } else if (has_lookahead && lookahead_angle_score > 0 && dist_robot_to_lookahead > kLookAheadDistance / 2 &&
             viewpoint_manager_->InLocalPlanningHorizon(local_path.nodes_[lookahead_i].position_))

  {
    lookahead_point = local_path.nodes_[lookahead_i].position_;
  } else {
    if (forward_angle_score > backward_angle_score) {
      if (forward_viewpoint_count > 0) {
        lookahead_point = forward_lookahead_point;
      } else {
        lookahead_point = backward_lookahead_point;
      }
    } else {
      if (backward_viewpoint_count > 0) {
        lookahead_point = backward_lookahead_point;
      } else {
        lookahead_point = forward_lookahead_point;
      }
    }
  }

  if ((lookahead_point == forward_lookahead_point && !forward_lookahead_point_in_los) ||
      (lookahead_point == backward_lookahead_point && !backward_lookahead_point_in_los)) 
  {
    lookahead_point_in_line_of_sight_ = false;
  } 
  else 
  {
    lookahead_point_in_line_of_sight_ = true;
  }

  lookahead_point_direction_ = lookahead_point - robot_position;
  lookahead_point_direction_.z() = 0.0;
  lookahead_point_direction_.normalize();

  pcl::PointXYZI point;
  point.x = lookahead_point.x();
  point.y = lookahead_point.y();
  point.z = lookahead_point.z();
  point.intensity = 1.0;
  lookahead_point_cloud_->cloud_->points.push_back(point);

  if (has_lookahead) {
    point.x = local_path.nodes_[lookahead_i].position_.x();
    point.y = local_path.nodes_[lookahead_i].position_.y();
    point.z = local_path.nodes_[lookahead_i].position_.z();
    point.intensity = 0;
    lookahead_point_cloud_->cloud_->points.push_back(point);
  }
  return true;
}

// Far 的 单个 方向的 路径 在 局部路径规划框中 获取 高分辨率的 路径
void SensorCoveragePlanner3D::GetLookAheadPoint_Globalpath_Far(const exploration_path_ns::ExplorationPath& global_path,
                                                               Eigen::Vector3d& lookahead_point)
{
  // 当前的 global_path 是单方向的 起点是机器人位置 终点是目标点
  // 需要使用局部规划框获得 globa_local_path（高分辨率的局部路径）做目标点选择

  exploration_path_ns::ExplorationPath global_local_path;
  RCLCPP_DEBUG(this->get_logger(), "计算全局局部路径 global_local_path");
  local_coverage_planner_->GetGlobalLocalPath(global_path, global_local_path);
  
  // 判定当前位置是否接近最后的导航点，接近则不扩展
  if (global_path.GetNodeNum() >= 2 && global_path.GetLength() < kLookAheadDistance)
  {
    is_close_to_goal_cell_ = true;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "global_local_path.size() = %zu", global_local_path.nodes_.size());
  
  // 从单向的全局导航路径和全局局部高分辨率路径中选择目标点
  GetLookAheadPoint_Globalpath_GlobalLocalpath(global_path, global_local_path, lookahead_point);
}

// 从单向的全局导航路径和全局局部高分辨率路径中选择目标点
void SensorCoveragePlanner3D::GetLookAheadPoint_Globalpath_GlobalLocalpath(
    const exploration_path_ns::ExplorationPath& global_path,
    const exploration_path_ns::ExplorationPath& global_local_path, Eigen::Vector3d& lookahead_point)
{
  // 如果局部路径太短则使用全局路径
  Eigen::Vector3d robot_position(robot_position_.x, robot_position_.y, robot_position_.z);
  bool global_local_path_too_short = true;  // 标记局部路径是否过短

  bool debug = false;
  if (debug)
  {
    RCLCPP_INFO(this->get_logger(), "当前最新位置");
    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", 
                robot_position[0], robot_position[1], robot_position[2]);

    RCLCPP_INFO(this->get_logger(), "全局路径起点");
    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f",
                global_path.nodes_.front().position_[0],
                global_path.nodes_.front().position_[1],
                global_path.nodes_.front().position_[2]);
                
    RCLCPP_INFO(this->get_logger(), "全局路径终点");
    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f",
                global_path.nodes_.back().position_[0],
                global_path.nodes_.back().position_[1],
                global_path.nodes_.back().position_[2]);
                
    RCLCPP_INFO(this->get_logger(), "全局路径长度: %f", global_path.GetLength());

    RCLCPP_INFO(this->get_logger(), "全局局部路径起点");
    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f",
                global_local_path.nodes_[0].position_[0],
                global_local_path.nodes_[0].position_[1],
                global_local_path.nodes_[0].position_[2]);
                
    RCLCPP_INFO(this->get_logger(), "全局局部路径终点");
    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f",
                global_local_path.nodes_.back().position_[0],
                global_local_path.nodes_.back().position_[1],
                global_local_path.nodes_.back().position_[2]);
                
    RCLCPP_INFO(this->get_logger(), "全局局部路径长度: %f", global_local_path.GetLength());
  }

  if (global_local_path.nodes_.size() > 2)
  {
    for (int i = 1; i < global_local_path.nodes_.size(); ++i)
    {
      double dist_to_robot = (robot_position - global_local_path.nodes_[i].position_).norm();
      if (dist_to_robot > 0.3)  // 距离阈值判断
      {
        global_local_path_too_short = false;
        break;
      }
    }
  }

  RCLCPP_DEBUG(this->get_logger(), "全局局部路径%s", 
              global_local_path_too_short ? "太短" : "足够长");

  // 判定使用全局规划的结果
  if (global_local_path.GetNodeNum() < 2 || global_local_path_too_short)
  {
    RCLCPP_DEBUG(this->get_logger(), "全局局部路径太短，使用全局far_path");
    double dist_from_robot = 0.0;
    
    for (int i = 1; i < global_path.nodes_.size(); ++i)
    {
      // 目标点是否在视线内
      bool in_line_of_sight = true;
      if (i < global_path.GetNodeNum() - 1)
      {
        in_line_of_sight = viewpoint_manager_->InCurrentFrameLineOfSight(
            global_path.nodes_[i + 1].position_);
      }
      
      // 累加距离
      dist_from_robot += (global_path.nodes_[i - 1].position_ - global_path.nodes_[i].position_).norm();

      // 选点逻辑
      if ((kUseLineOfSightLookAheadPoint && !in_line_of_sight) && dist_from_robot > kLookAheadDistance / 5)
      {
        lookahead_point = global_path.nodes_[i].position_;
        RCLCPP_DEBUG(this->get_logger(), "下一个点不在视线内，且距离有一定长度");
        return;
      }
      else if (dist_from_robot > kLookAheadDistance)
      {
        lookahead_point = global_path.nodes_[i].position_;
        RCLCPP_DEBUG(this->get_logger(), "找到距离足够的全局目标点");
        return;
      }
    }
    
    lookahead_point = global_path.nodes_.back().position_;
    RCLCPP_DEBUG(this->get_logger(), "以最后目标点为全局目标点");
    return;
  }

  // 使用全局局部路径
  RCLCPP_DEBUG(this->get_logger(), "使用全局局部路径 global_local_path");
  bool forward_lookahead_point_in_los = true;
  double length_from_robot = 0;
  
  for (int i = 1; i < global_local_path.GetNodeNum(); ++i)
  {
    // 累计路径长度
    length_from_robot += (global_local_path.nodes_[i].position_ - global_local_path.nodes_[i - 1].position_).norm();
                         
    // 计算直线距离
    double dist_to_robot = (global_local_path.nodes_[i].position_ - robot_position).norm();
    
    // 视线判断
    bool in_line_of_sight = true;
    if (i < global_local_path.GetNodeNum() - 1)
    {
      in_line_of_sight = viewpoint_manager_->InCurrentFrameLineOfSight(global_local_path.nodes_[i + 1].position_);
    }
    
    // 前视点选择条件
    if ((length_from_robot > kLookAheadDistance || (kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
         global_local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
         global_local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_START ||
         global_local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_END ||
         i == global_local_path.GetNodeNum() - 1))
    {
      if (kUseLineOfSightLookAheadPoint && !in_line_of_sight)
      {
        forward_lookahead_point_in_los = false;
      }
      
      lookahead_point = global_local_path.nodes_[i].position_;
      RCLCPP_DEBUG(this->get_logger(), "找到满足条件的局部目标点");
      return;
    }
  }
  
  // 保险机制，使用最后一个点
  lookahead_point = global_local_path.nodes_.back().position_;
  RCLCPP_DEBUG(this->get_logger(), "使用保险的局部目标点");
  return;
}

// 修改：从局部路径得到前视点并发布
bool SensorCoveragePlanner3D::GetLookAheadPoint_Localpath(const exploration_path_ns::ExplorationPath& local_path,
                                                          Eigen::Vector3d& lookahead_point)
{
  Eigen::Vector3d robot_position(robot_position_.x, robot_position_.y, robot_position_.z);  // 机器人当前位置
  geometry_msgs::msg::Point robot_position_Point;  // 机器人当前位置（ROS2消息格式）
  robot_position_Point.x = robot_position[0];
  robot_position_Point.y = robot_position[1];
  robot_position_Point.z = robot_position[2];
  
  std::map<int, geometry_msgs::msg::Point> other_robot_position_map = other_robot_position_map_;
  bool has_lookahead = false;  // 已经有前视点
  bool dir = true;
  int robot_i = 0;             // 机器人位置在局部路径上的点索引
  int lookahead_i = 0;         // 前视点在局部路径规划上的索引
  
  if (local_path.nodes_.empty())  // 路径为空时跳过
  {
    return false;
  }
  
  for (int i = 0; i < local_path.nodes_.size(); ++i)
  {
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::ROBOT)  // 找到机器人位置的局部规划路线点
    {
      robot_i = i;
    }
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOOKAHEAD_POINT)  // 找到前视点
    {
      has_lookahead = true;
      lookahead_i = i;
    }
  }

  int forward_viewpoint_count = 0;   // 向前视点个数
  int backward_viewpoint_count = 0;  // 向后视点个数
  bool local_loop = false;           // 局部循环标志

  // 判断局部路径是否为循环路径
  if (local_path.nodes_.front() == local_path.nodes_.back() &&
      local_path.nodes_.front().type_ == exploration_path_ns::NodeType::ROBOT)
  {
    local_loop = true;
    RCLCPP_DEBUG(this->get_logger(), "检测到局部循环路径");
  }

  // 统计向前方向的局部视点数量
  if (local_loop)
  { 
    robot_i = 0;  // 循环路径时重置机器人位置索引
  }
  for (int i = robot_i + 1; i < local_path.GetNodeNum(); ++i)
  {
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT)
    {
      forward_viewpoint_count++;
    }
  }

  // 统计向后方向的局部视点数量
  if (local_loop) 
  {
    robot_i = local_path.nodes_.size() - 1;  // 循环路径时重置机器人位置索引
  }
  for (int i = robot_i - 1; i >= 0; --i)
  {
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT)
    {
      backward_viewpoint_count++;
    }
  }

  RCLCPP_DEBUG(this->get_logger(), "前向局部视点数量: %d", forward_viewpoint_count);
  RCLCPP_DEBUG(this->get_logger(), "后向局部视点数量: %d", backward_viewpoint_count);

  // 初始化前向和后向视点为机器人当前位置
  Eigen::Vector3d forward_lookahead_point = robot_position;
  Eigen::Vector3d backward_lookahead_point = robot_position;
  bool has_forward = false;
  bool has_backward = false;

  if (local_loop)
  {
    robot_i = 0;
  }
  bool forward_lookahead_point_in_los = true;
  bool backward_lookahead_point_in_los = true;
  double length_from_robot = 0.0;
  for (int i = robot_i + 1; i < local_path.GetNodeNum(); ++i)
  {
    length_from_robot += (local_path.nodes_[i].position_ - local_path.nodes_[i - 1].position_).norm();
    double dist_to_robot = (local_path.nodes_[i].position_ - robot_position).norm();
    
    bool in_line_of_sight = true;
    if (i < local_path.GetNodeNum() - 1)
    {
      in_line_of_sight = viewpoint_manager_->InCurrentFrameLineOfSight(local_path.nodes_[i + 1].position_);
    }
    if ((length_from_robot > kLookAheadDistance || (kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_START ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_END ||
         i == local_path.GetNodeNum() - 1))
    {
      if (kUseLineOfSightLookAheadPoint && !in_line_of_sight)
      {
        forward_lookahead_point_in_los = false;
      }
      forward_lookahead_point = local_path.nodes_[i].position_;
      has_forward = true;
      break;
    }
  }

  if (local_loop) 
  {
    robot_i = local_path.nodes_.size() - 1;
  }
  length_from_robot = 0.0;
  for (int i = robot_i - 1; i >= 0; --i)
  {
    length_from_robot += (local_path.nodes_[i].position_ - local_path.nodes_[i + 1].position_).norm();   
    double dist_to_robot = (local_path.nodes_[i].position_ - robot_position).norm();
    bool in_line_of_sight = true;
    
    if (i > 0)
    {
      in_line_of_sight = viewpoint_manager_->InCurrentFrameLineOfSight(local_path.nodes_[i - 1].position_);
    }
    
    if ((length_from_robot > kLookAheadDistance || (kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_START ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_END || i == 0))
    {
      if (kUseLineOfSightLookAheadPoint && !in_line_of_sight)
      {
        backward_lookahead_point_in_los = false;
      }
      backward_lookahead_point = local_path.nodes_[i].position_;
      has_backward = true;
      break;
    }
  }

  if (forward_viewpoint_count > 0 && !has_forward)
  {
    RCLCPP_ERROR(this->get_logger(), "前向视点数大于0但未找到前向点");
    exit(1);
  }
  if (backward_viewpoint_count > 0 && !has_backward)
  {
    RCLCPP_ERROR(this->get_logger(), "后向视点数大于0但未找到后向点");
    exit(1);
  }

  double dx = lookahead_point_direction_.x();
  double dy = lookahead_point_direction_.y();
  double forward_angle_score = -2;
  double backward_angle_score = -2;
  double lookahead_angle_score = -2;
  double dist_robot_to_lookahead = 0.0;
  int other_robot_num = other_robot_position_map_.size();

  // 计算前向点分数
  if (has_forward)
  {
    Eigen::Vector3d forward_diff = forward_lookahead_point - robot_position;
    forward_diff.z() = 0.0;
    forward_diff = forward_diff.normalized(); // 修改 2025.10.23
    forward_angle_score = dx * forward_diff.x() + dy * forward_diff.y();

    double distance_score = 0;
    geometry_msgs::msg::Point forward_lookahead_point_;
    forward_lookahead_point_.x = forward_lookahead_point[0];
    forward_lookahead_point_.y = forward_lookahead_point[1];
    forward_lookahead_point_.z = forward_lookahead_point[2];

    for (auto& itr : other_robot_position_map_)
    {
      double distance = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(forward_lookahead_point_, itr.second);
      if (distance < kLocal_planning_range)
      {
        double score = (distance / kLocal_planning_range) / other_robot_num;
        distance_score += score;
      }
      else
      {
        double score = 1.0 / other_robot_num;
        distance_score += score;
      }
    }
    forward_angle_score += distance_score;
  }

  if (has_backward)
  {
    Eigen::Vector3d backward_diff = backward_lookahead_point - robot_position;
    backward_diff.z() = 0.0;
    backward_diff = backward_diff.normalized(); // 修改 2025.10.23
    backward_angle_score = dx * backward_diff.x() + dy * backward_diff.y();

    double distance_score = 0;
    geometry_msgs::msg::Point backward_lookahead_point_;
    backward_lookahead_point_.x = backward_lookahead_point[0];
    backward_lookahead_point_.y = backward_lookahead_point[1];
    backward_lookahead_point_.z = backward_lookahead_point[2];

    for (auto& itr : other_robot_position_map_)
    {
      robot_position_Point;
      double distance = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
          backward_lookahead_point_, itr.second);
      
      if (distance < kLocal_planning_range)
      {
        double score = (distance / kLocal_planning_range) / other_robot_num;
        distance_score += score;
      }
      else
      {
        double score = 1.0 / other_robot_num;
        distance_score += score;
      }
    }
    backward_angle_score += distance_score;
  }

  if (has_lookahead)
  {
    Eigen::Vector3d prev_lookahead_point = local_path.nodes_[lookahead_i].position_;
    dist_robot_to_lookahead = (robot_position - prev_lookahead_point).norm();
    
    Eigen::Vector3d diff = prev_lookahead_point - robot_position;
    diff.z() = 0.0;
    diff = diff.normalized(); // 修改 2025.10.23
    lookahead_angle_score = dx * diff.x() + dy * diff.y();

    double distance_score = 0;
    geometry_msgs::msg::Point prev_lookahead_point_;
    prev_lookahead_point_.x = prev_lookahead_point[0];
    prev_lookahead_point_.y = prev_lookahead_point[1];
    prev_lookahead_point_.z = prev_lookahead_point[2];

    for (auto& itr : other_robot_position_map_)
    {
      robot_position_Point;
      double distance = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
          prev_lookahead_point_, itr.second);
      
      if (distance < kLocal_planning_range)
      {
        double score = (distance / kLocal_planning_range) / other_robot_num;
        distance_score += score;
      }
      else
      {
        double score = 1.0 / other_robot_num;
        distance_score += score;
      }
    }
    lookahead_angle_score += distance_score;
  }

  lookahead_point_cloud_->cloud_->clear();
  if (forward_viewpoint_count == 0 && backward_viewpoint_count == 0)
  {
    RCLCPP_INFO(this->get_logger(), "机器人需要重定位");
  }
  else
  {
    relocation_ = false;
  }

  if (relocation_)
  {
    if (use_momentum_ && kUseMomentum)
    {
      if(forward_angle_score > backward_angle_score)
      {
        lookahead_point = forward_lookahead_point;
      }
      else
      {
        lookahead_point = backward_lookahead_point;
      }
    }
    else
    {
      lookahead_point = (forward_angle_score > backward_angle_score) ? forward_lookahead_point : backward_lookahead_point;
      RCLCPP_INFO(this->get_logger(), "机器人: %s 基于角度分数选择方向 (前向: %.2f, 后向: %.2f)",
                robot_name.c_str(), forward_angle_score, backward_angle_score);
    }
  }

  else if (has_lookahead && lookahead_angle_score > 0 && dist_robot_to_lookahead > kLookAheadDistance / 2 &&
           viewpoint_manager_->InLocalPlanningHorizon(local_path.nodes_[lookahead_i].position_))
  {
    lookahead_point = local_path.nodes_[lookahead_i].position_;
    RCLCPP_INFO(this->get_logger(), "机器人: %s 选择前一次目标视点", robot_name.c_str());
  }

  else
  {
    if (forward_angle_score > backward_angle_score)
    {
      if (forward_viewpoint_count > 0)
      {
        lookahead_point = forward_lookahead_point;
        RCLCPP_INFO(this->get_logger(), "机器人: %s 选择向前视点", robot_name.c_str());
      }
      else
      {
        lookahead_point = backward_lookahead_point;
        RCLCPP_INFO(this->get_logger(), "机器人: %s 选择向后视点", robot_name.c_str());
      }
    }
    else
    {
      if (backward_viewpoint_count > 0)
      {
        lookahead_point = backward_lookahead_point;
        RCLCPP_INFO(this->get_logger(), "机器人: %s 选择向后视点", robot_name.c_str());
      }
      else
      {
        lookahead_point = forward_lookahead_point;
        RCLCPP_INFO(this->get_logger(), "机器人: %s 选择向前视点", robot_name.c_str());
      }
    }
  }

  // 更新视线内标志
  if ((lookahead_point == forward_lookahead_point && !forward_lookahead_point_in_los) ||
      (lookahead_point == backward_lookahead_point && !backward_lookahead_point_in_los))
  {
    lookahead_point_in_line_of_sight_ = false;
  }
  else
  {
    lookahead_point_in_line_of_sight_ = true;
  }

  // 更新前视点方向
  lookahead_point_direction_ = lookahead_point - robot_position;
  lookahead_point_direction_.z() = 0.0;
  lookahead_point_direction_.normalize();

  // 添加前视点到点云用于可视化
  pcl::PointXYZI point;
  point.x = lookahead_point.x();
  point.y = lookahead_point.y();
  point.z = lookahead_point.z();
  point.intensity = 1.0;
  lookahead_point_cloud_->cloud_->points.push_back(point);

  // 如果有上一个前视点，也添加到点云
  if (has_lookahead)
  {
    point.x = local_path.nodes_[lookahead_i].position_.x();
    point.y = local_path.nodes_[lookahead_i].position_.y();
    point.z = local_path.nodes_[lookahead_i].position_.z();
    point.intensity = 0;
    lookahead_point_cloud_->cloud_->points.push_back(point);
  }

  return true;
}

void SensorCoveragePlanner3D::PublishWaypoint() {
  geometry_msgs::msg::PointStamped waypoint;
  if (exploration_finished_ && near_home_ && kRushHome) 
  {
    waypoint.point.x = initial_position_.x();
    waypoint.point.y = initial_position_.y();
    waypoint.point.z = initial_position_.z();
  } 
  else 
  {
    double dx = lookahead_point_.x() - robot_position_.x;
    double dy = lookahead_point_.y() - robot_position_.y;
    double r = sqrt(dx * dx + dy * dy);
    double extend_dist = lookahead_point_in_line_of_sight_ ? kExtendWayPointDistanceBig : kExtendWayPointDistanceSmall;
    if (r < extend_dist && kExtendWayPoint && !is_close_to_goal_cell_) {
      dx = dx / r * extend_dist;
      dy = dy / r * extend_dist;
    }
    waypoint.point.x = dx + robot_position_.x;
    waypoint.point.y = dy + robot_position_.y;
    waypoint.point.z = lookahead_point_.z();
  }
  misc_utils_ns::Publish(shared_from_this(), waypoint_pub_, waypoint, robot_name + "/" + kWorldFrameID);
  geometry_msgs::msg::PoseStamped move_base_goal;
  move_base_goal.pose.position = waypoint.point;
  move_base_goal.pose.orientation.w = 1;

  misc_utils_ns::Publish(shared_from_this(), move_base_goal_pub_, move_base_goal, "map");
}

void SensorCoveragePlanner3D::PublishRuntime() {
  local_viewpoint_sampling_runtime_ = local_coverage_planner_->GetViewPointSamplingRuntime() / 1000;
  local_path_finding_runtime_ = (local_coverage_planner_->GetFindPathRuntime() + local_coverage_planner_->GetTSPRuntime()) / 1000;

  std_msgs::msg::Int32MultiArray runtime_breakdown_msg;
  runtime_breakdown_msg.data.clear();
  runtime_breakdown_msg.data.push_back(update_representation_runtime_);
  runtime_breakdown_msg.data.push_back(local_viewpoint_sampling_runtime_);
  runtime_breakdown_msg.data.push_back(local_path_finding_runtime_);
  runtime_breakdown_msg.data.push_back(global_planning_runtime_);
  runtime_breakdown_msg.data.push_back(trajectory_optimization_runtime_);
  runtime_breakdown_msg.data.push_back(overall_runtime_);
  runtime_breakdown_pub_->publish(runtime_breakdown_msg);

  float runtime = 0;
  if (!exploration_finished_ && kNoExplorationReturnHome) {
    for (int i = 0; i < runtime_breakdown_msg.data.size() - 1; i++) {
      runtime += runtime_breakdown_msg.data[i];
    }
  }

  std_msgs::msg::Float32 runtime_msg;
  runtime_msg.data = runtime / 1000.0;
  runtime_pub_->publish(runtime_msg);
}

double SensorCoveragePlanner3D::GetRobotToHomeDistance() {
  Eigen::Vector3d robot_position(robot_position_.x, robot_position_.y, robot_position_.z);
  return (robot_position - initial_position_).norm();
}

void SensorCoveragePlanner3D::PublishExplorationState() {
  std_msgs::msg::Bool exploration_finished_msg;
  exploration_finished_msg.data = exploration_finished_;
  exploration_finish_pub_->publish(exploration_finished_msg);
}

void SensorCoveragePlanner3D::PrintExplorationStatus(std::string status, bool clear_last_line) 
{
  if (clear_last_line) {
    printf(cursup);
    printf(cursclean);
    printf(cursup);
    printf(cursclean);
  }
  std::cout << std::endl << "\033[1;32m" << status << "\033[0m" << std::endl;
}

void SensorCoveragePlanner3D::CountDirectionChange() {
  Eigen::Vector3d current_moving_direction_ =
      Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z) -
      Eigen::Vector3d(last_robot_position_.x, last_robot_position_.y,
                      last_robot_position_.z);

  if (current_moving_direction_.norm() > 0.5) {
    if (moving_direction_.dot(current_moving_direction_) < 0) {
      direction_change_count_++;
      direction_no_change_count_ = 0;
      if (direction_change_count_ > kDirectionChangeCounterThr) {
        if (!use_momentum_) {
          momentum_activation_count_++;
        }
        use_momentum_ = true;
      }
    } else {
      direction_no_change_count_++;
      if (direction_no_change_count_ > kDirectionNoChangeCounterThr) {
        direction_change_count_ = 0;
        use_momentum_ = false;
      }
    }
    moving_direction_ = current_moving_direction_;
  }
  last_robot_position_ = robot_position_;

  std_msgs::msg::Int32 momentum_activation_count_msg;
  momentum_activation_count_msg.data = momentum_activation_count_;
  momentum_activation_count_pub_->publish(momentum_activation_count_msg);
}

// 添加
void SensorCoveragePlanner3D::AddLocalSharedMergerInfor(tare_planner::msg::SharedMergerInfor& Shared_Infor)
{
  for (auto& node : Shared_Infor.new_node_set)
  {
    Shared_Infor_map_[Shared_Infor.robot_id].new_node_set.push_back(node);
  }
  // 添加边
  for (auto& add_edge : Shared_Infor.add_edge_set)
  {
    Shared_Infor_map_[Shared_Infor.robot_id].add_edge_set.push_back(add_edge);
  }
  //删除边
  for (auto& delete_edge : Shared_Infor.delete_edge_set)
  {
    Shared_Infor_map_[Shared_Infor.robot_id].delete_edge_set.push_back(delete_edge);
  }
}

void SensorCoveragePlanner3D::execute_grid_merger_graph()
{
  if (!kAutoStart && !start_exploration_)
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for start signal");
    return;
  }
  
  Timer overall_processing_timer("overall processing");

  update_representation_runtime_ = 0;
  local_viewpoint_sampling_runtime_ = 0;
  local_path_finding_runtime_ = 0;
  global_planning_runtime_ = 0;
  trajectory_optimization_runtime_ = 0;
  overall_runtime_ = 0;

  if (!initialized_)  // 等待初始化完成（获取车辆当前状态）
  {
    SendInitialWaypoint();  // 发送当前航点
    start_time_ = this->get_clock()->now();
    global_direction_switch_time_ = this->get_clock()->now();  // 全局方向切换时间
    return;
  }

  overall_processing_timer.Start();  // 开始计时
  
  RCLCPP_DEBUG(this->get_logger(), "++++++++++++++++++++++++++++++ start ++++++++++++++++++++++++++++++");
  
  if (keypose_cloud_update_)  
  {
    keypose_cloud_update_ = false;

    CountDirectionChange();  

    misc_utils_ns::Timer update_representation_timer("update representation");
    update_representation_timer.Start();
    UpdateGlobalRepresentation();  
    int viewpoint_candidate_count = UpdateViewPoints();
    if (viewpoint_candidate_count == 0)
    {
      RCLCPP_WARN(this->get_logger(), "Cannot get candidate viewpoints, skipping this round");
      return;
    }

    UpdateKeyposeGraph();  

    int uncovered_point_num = 0;
    int uncovered_frontier_point_num = 0;
    
    if (!exploration_finished_ || !kNoExplorationReturnHome)  
    {
      UpdateViewPointCoverage(); 
      UpdateCoveredAreas(uncovered_point_num, uncovered_frontier_point_num);
      RCLCPP_DEBUG(this->get_logger(), "uncovered_point_num: %d", uncovered_point_num);
      RCLCPP_DEBUG(this->get_logger(), "uncovered_frontier_point_num: %d", uncovered_frontier_point_num);
    }
    else
    {
      viewpoint_manager_->ResetViewPointCoverage();  
    }

    update_representation_timer.Stop(false);
    update_representation_runtime_ += update_representation_timer.GetDuration("ms");
    GlobalStatuUpdate();

    UpdateMergerGraph();
    exploration_path_ns::ExplorationPath local_path;
    exploration_path_ns::ExplorationPath local_path_sort;
    std::vector<exploration_path_ns::ExplorationPath> near_localcoverage_subgrid_paths;
    Get_subgrid_paths(near_localcoverage_subgrid_paths, keypose_graph_);
    LocalPlanning_opt(uncovered_point_num, uncovered_frontier_point_num, near_localcoverage_subgrid_paths, local_path);
    local_path_sort = SortLocalPath(local_path);
    bool is_global_tsp = Is_Global_Planner(local_path_sort);
    last_isglobal_tsp_ = is_global_tsp;
    std::vector<int> global_cell_tsp_order;            
    exploration_path_ns::ExplorationPath global_path;  
    GlobalPlanning_grid_merger_graph(global_cell_tsp_order, global_path, is_global_tsp);
    robot_statu_ = grid_world_->GetRobotStatu();
    tare_planner::msg::ReturnHome Return_home_msg;
    Return_home_msg.robot_id = robot_id_;
    
    if (robot_statu_ == grid_world_ns::RobotStatus::Return_home)
    {
      Return_home_msg.is_returning_home = true;
      is_returning_home_pub_->publish(Return_home_msg);
    }
    else
    {
      Return_home_msg.is_returning_home = false;
      is_returning_home_pub_->publish(Return_home_msg);
    }

    bool all_return_home = false;
    int count = 0;
    for (int i = 0; i < is_returning_home_list.size(); i++)
    {
      if (is_returning_home_list[i] == true)
      {
        count++;
      }
    }
    if (count == is_returning_home_list.size())
    {
      all_return_home = true;
    }
    RCLCPP_DEBUG(this->get_logger(), "all_return_home: %s", all_return_home ? "true" : "false");

    near_home_ = GetRobotToHomeDistance() < kRushHomeDist;       // 靠近起点
    at_home_ = GetRobotToHomeDistance() < kAtHomeDistThreshold;  // 在起点位置

    if (all_return_home &&
        local_coverage_planner_->IsLocalCoverageComplete() &&  // 局部规划完成（无候选点）
        (this->get_clock()->now() - start_time_).seconds() > 5)
    {
      if (!exploration_finished_)
      {
        PrintExplorationStatus("Exploration completed, returning home", false);
      }
      exploration_finished_ = true;  // 探索完成
    }

    if (exploration_finished_ && at_home_ && !stopped_)
    {
      PrintExplorationStatus("Return home completed", false);
      stopped_ = true;  
    }
    
    PublishExplorationState();

    is_close_to_goal_cell_ = false;  // 使用 扩展

    if (robot_statu_ == grid_world_ns::RobotStatus::Far_planner)  
    {
      GetLookAheadPoint_Globalpath_Far(global_path, lookahead_point_);
      RCLCPP_INFO(this->get_logger(), "robot_statu_= Far_planner");
    }
    else  
    {
      if (robot_statu_ == grid_world_ns::RobotStatus::Exploring)
      {
        is_exploring_ = true;
        lookahead_point_update_ = GetLookAheadPoint_Localpath(local_path_sort, lookahead_point_);
        RCLCPP_INFO(this->get_logger(), "robot_statu_= Exploring");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "robot_statu_= Global_tsp || Return_home");
        
        exploration_path_ns::ExplorationPath global_local_path;
        bool use_local = false;
        
        if (global_path.nodes_.size() != 0)
        {
          use_local = local_coverage_planner_->GetGlobalLocalPath_single(global_path, global_local_path);
          exploration_path_ = ConcatenateGlobalLocalPath(global_path, global_local_path);
          lookahead_point_update_ = GetLookAheadPoint(exploration_path_, global_path, lookahead_point_);
        }
      }
    }
    bool debug = false;
    if (debug)
    {
      RCLCPP_DEBUG(this->get_logger(), "====== wagpoint ======");
      RCLCPP_DEBUG(this->get_logger(), "====== x= %f", lookahead_point_.x());
      RCLCPP_DEBUG(this->get_logger(), "====== y= %f", lookahead_point_.y());
      RCLCPP_DEBUG(this->get_logger(), "====== z= %f", lookahead_point_.z());
      RCLCPP_DEBUG(this->get_logger(), "====== 当前位置点 ======");
      RCLCPP_DEBUG(this->get_logger(), "======cur_posisition =");
      RCLCPP_DEBUG(this->get_logger(), "====== x= %f", robot_position_.x);
      RCLCPP_DEBUG(this->get_logger(), "====== y= %f", robot_position_.y);
      RCLCPP_DEBUG(this->get_logger(), "====== z= %f", robot_position_.z);
    }
    
    PublishWaypoint();  // 发布 waypint
    
    overall_processing_timer.Stop(false);
    overall_runtime_ = overall_processing_timer.GetDuration("ms");  // 记录运行时间

    // 可视化相关
    visualizer_->GetGlobalSubspaceMarker(grid_world_, global_cell_tsp_order);
    Eigen::Vector3d viewpoint_origin = viewpoint_manager_->GetOrigin();
    visualizer_->GetLocalPlanningHorizonMarker(
        viewpoint_origin.x(), viewpoint_origin.y(), robot_position_.z);
    visualizer_->PublishMarkers();

    // 发布规划结果可视化和运行时间
    PublishLocalPlanningVisualization(local_path);
    PublishGlobalPlanningVisualization(global_path, local_path);
    PublishRuntime();
  }
}

bool SensorCoveragePlanner3D::GetLocalPlanningType(const exploration_path_ns::ExplorationPath& global_path)
{
  bool is_exploring = grid_world_->GetLocalPlanningType(global_path);
  return is_exploring;
}

} // namespace sensor_coverage_planner_3d_ns
