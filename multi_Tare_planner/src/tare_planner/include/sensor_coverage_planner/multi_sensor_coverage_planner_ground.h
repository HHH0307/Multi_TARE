/**
 * @file sensor_coverage_planner_ground.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that does the job of exploration
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Core>
// ROS
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/transform_datatypes.h>
// PCL
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
// Third parties
#include <utils/misc_utils.h>
#include <utils/pointcloud_utils.h>
// Components
#include "keypose_graph/keypose_graph.h"
#include "merger_graph/merger_graph.h"
#include "planning_env/planning_env.h"
#include "viewpoint_manager/viewpoint_manager.h"
#include "grid_world/grid_world.h"
#include "exploration_path/exploration_path.h"
#include "local_coverage_planner/local_coverage_planner.h"
#include "tare_visualizer/multi_tare_visualizer.h"
#include "rolling_occupancy_grid/rolling_occupancy_grid.h"

// 新增 服务定义的数据
#include "tare_planner/srv/request_path.hpp"
// 新增 话题定义的数据
#include "tare_planner/msg/subgraph.hpp"
#include "tare_planner/msg/return_home.hpp"
#include "tare_planner/msg/shared_merger_infor.hpp"

#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"

namespace sensor_coverage_planner_3d_ns 
{
const std::string kWorldFrameID = "map";
typedef pcl::PointXYZRGBNormal PlannerCloudPointType;
typedef pcl::PointCloud<PlannerCloudPointType> PlannerCloudType;
typedef misc_utils_ns::Timer Timer;

class SensorCoveragePlanner3D : public rclcpp::Node {
public:
  explicit SensorCoveragePlanner3D();
  bool initialize();
  void execute();
  void execute_pro();
  void execute_max();
  void execute_grid_graph();
  void execute_grid_graph_opt();
  void execute_grid_merger_graph();
  ~SensorCoveragePlanner3D() = default;

private:
  // String
  std::string robot_name;
  std::string allocation_strategy_;    // 新增的分配策略     MinDis Greedy MinPos Mdvrp
  std::string robot_public_name_; // 新增
  std::string sub_start_exploration_topic_;
  std::string sub_keypose_topic_;
  std::string sub_state_estimation_topic_;
  std::string sub_registered_scan_topic_;
  std::string sub_terrain_map_topic_;
  std::string sub_terrain_map_ext_topic_;
  std::string sub_coverage_boundary_topic_;
  std::string sub_viewpoint_boundary_topic_;
  std::string sub_nogo_boundary_topic_;
  std::string sub_joystick_topic_;
  std::string sub_reset_waypoint_topic_;

  std::string pub_exploration_finish_topic_;
  std::string pub_runtime_breakdown_topic_;
  std::string pub_runtime_topic_;
  std::string pub_waypoint_topic_;
  std::string pub_momentum_activation_count_topic_;

  // Bool
  bool kAutoStart;
  bool kRushHome;
  bool kUseTerrainHeight;
  bool kCheckTerrainCollision;
  bool kExtendWayPoint;
  bool kUseLineOfSightLookAheadPoint;
  bool kNoExplorationReturnHome;
  bool kUseMomentum;

  // Double
  double kKeyposeCloudDwzFilterLeafSize;
  double kRushHomeDist;
  double kAtHomeDistThreshold;
  double kTerrainCollisionThreshold;
  double kLookAheadDistance;
  double kExtendWayPointDistanceBig;
  double kExtendWayPointDistanceSmall;
  //新增网格大小
  double kLocal_planning_range;
  double kCellSize_;

  // Int
  int kDirectionChangeCounterThr;
  int kDirectionNoChangeCounterThr;

  // 新增
  int robot_num;
  std::vector<bool> is_returning_home_list;
  int kResetWaypointJoystickAxesID;

  std::shared_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> keypose_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>> registered_scan_stack_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> registered_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> large_terrain_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> terrain_collision_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> terrain_ext_collision_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> viewpoint_vis_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> grid_world_vis_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> selected_viewpoint_vis_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> exploring_cell_vis_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> exploration_path_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> collision_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> lookahead_point_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> keypose_graph_vis_cloud_;
  // 新增  拼接图 的点云
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> merger_graph_vis_cloud_;  //关键位姿图  点云 
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> viewpoint_in_collision_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> point_cloud_manager_neighbor_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> reordered_global_subspace_cloud_;
  //新增点云  显示
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> MTSP_grid_graph_vis_cloud_;  //可视化MTSP网格图  点云

  nav_msgs::msg::Odometry keypose_;
  geometry_msgs::msg::Point robot_position_;
  geometry_msgs::msg::Point last_robot_position_;
  lidar_model_ns::LiDARModel robot_viewpoint_;
  exploration_path_ns::ExplorationPath exploration_path_;
  Eigen::Vector3d lookahead_point_;
  Eigen::Vector3d lookahead_point_direction_;
  Eigen::Vector3d moving_direction_;
  double robot_yaw_;
  bool moving_forward_;
  std::vector<Eigen::Vector3d> visited_positions_;
  exploration_path_ns::ExplorationPath historical_path_;
  int cur_keypose_node_ind_;
  Eigen::Vector3d initial_position_;

  std::shared_ptr<keypose_graph_ns::KeyposeGraph> keypose_graph_;
  //  新增    拼接图    merger_graph_  继承于   keypose_graph_     改动  使用  ikd 树做搜索  加快
  std::shared_ptr<merger_graph_ns::MergerGraph> merger_graph_;  // 全局的拼接图  接受所有  机器人的  keypose_graph_
  std::shared_ptr<planning_env_ns::PlanningEnv> planning_env_;
  std::shared_ptr<viewpoint_manager_ns::ViewPointManager> viewpoint_manager_;
  std::shared_ptr<local_coverage_planner_ns::LocalCoveragePlanner> local_coverage_planner_;
  std::shared_ptr<grid_world_ns::GridWorld> grid_world_;
  std::shared_ptr<tare_visualizer_ns::TAREVisualizer> visualizer_;

  std::shared_ptr<misc_utils_ns::Marker> keypose_graph_node_marker_;
  std::shared_ptr<misc_utils_ns::Marker> keypose_graph_edge_marker_;
  //  新增   拼接图的可视化
  std::shared_ptr<misc_utils_ns::Marker> merger_graph_node_marker_;
  std::shared_ptr<misc_utils_ns::Marker> merger_graph_edge_marker_;

  std::shared_ptr<misc_utils_ns::Marker> nogo_boundary_marker_;
  std::shared_ptr<misc_utils_ns::Marker> grid_world_marker_;

  //新增   子网格融合图      的  可视化
  std::shared_ptr<misc_utils_ns::Marker> MTSP_grid_graph_node_marker_;
  std::shared_ptr<misc_utils_ns::Marker> MTSP_grid_graph_edge_marker_;

  bool keypose_cloud_update_;
  bool initialized_;
  bool lookahead_point_update_;
  bool relocation_;
  bool start_exploration_;
  bool exploration_finished_;
  bool near_home_;
  bool at_home_;
  bool stopped_;
  bool test_point_update_;
  bool viewpoint_ind_update_;
  bool step_;
  bool use_momentum_;
  bool lookahead_point_in_line_of_sight_;
  bool last_isglobal_tsp_;//上一次是否是全局路径

  bool reset_waypoint_;
  pointcloud_utils_ns::PointCloudDownsizer<pcl::PointXYZ> pointcloud_downsizer_;
  //新增标志量
  // 新增一个导航  是否 靠近目标点网格 标记  
  // 当靠近  目标点网格时  使用  屏蔽  目标点  拓展
  // 使得可以精确到达其他机器人  发现的网格点位置  防止在目标网格左右摆。
  bool is_close_to_goal_cell_;

  int update_representation_runtime_;
  int local_viewpoint_sampling_runtime_;
  int local_path_finding_runtime_;
  int global_planning_runtime_;
  int trajectory_optimization_runtime_;
  int overall_runtime_;
  int registered_cloud_count_;
  int keypose_count_;
  int direction_change_count_;
  int direction_no_change_count_;
  int momentum_activation_count_;

  bool is_exploring_;
  grid_world_ns::RobotStatus robot_statu_;

  //新增 字典，存更新网格世界  编码和状态<网格单元编号,网格状态>
  std::map<int,int>  Update_Grid_World_ID_and_Statu_;
  //新增容器   存储  本地发现的需要探索的单元   Cell
  std::vector<int>  Cur_discover_exploring_cells;  //这个单元  发现的探索 单元
  //新增字典  存储   MTSP  子图    <机器人名，对应当前子图数据>
  std::map<int,std::vector<double>>  MTSP_subgrapher_map_;     //这个  在内部使用之后  就清空
  //新增字典   存储   其他机器人位置
  std::map<int,geometry_msgs::msg::Point>  other_robot_position_map_;
  //新增发布目标点
  geometry_msgs::msg::PointStamped last_goal;

  //新增 字典 存储其他机器人  邻接网格信息
  //  使用  机器人ID 查询  ，再使用网格 index  查询  累计的 网格结点
  std::map<int,std::map<int, tare_planner::msg::Subnode>>  robot_cell_subnode_;
  //新增字典   存储   其他 机器人   历史  位置
  std::map<int,std::vector<geometry_msgs::msg::Point>> other_robot_history_position_;
  int robot_id_;

  // 新增  接受一次的数据   用set 还是map?  map吧不用考虑个数
  std::unordered_map<int,tare_planner::msg::SharedMergerInfor> Shared_Infor_map_;
  // 新增     一次规划  维护  的数据
  tare_planner::msg::MergerGraphEdge MergerGraphEdge_Once_sub_;


  rclcpp::Time start_time_;
  rclcpp::Time global_direction_switch_time_;
  double reset_waypoint_joystick_axis_value_;

  rclcpp::TimerBase::SharedPtr execution_timer_;

  // ROS subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exploration_start_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr registered_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_map_ext_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_estimation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr coverage_boundary_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr viewpoint_boundary_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr nogo_boundary_sub_;

  //新增接受      更新的世界网格 状态
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr NewGridWorldCell_and_status_sub_;
  //接受   求解MTSP问题的子图
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr MTSP_SubGrapher_sub_;
  //新增  接受网格子图
  rclcpp::Subscription<tare_planner::msg::Subgraph>::SharedPtr Subgraph_sub_;
  //新增接受  是否返回起点
  rclcpp::Subscription<tare_planner::msg::ReturnHome>::SharedPtr is_returning_home_sub_;
  //  新增  接受  Shared_Infor_
  rclcpp::Subscription<tare_planner::msg::SharedMergerInfor>::SharedPtr Shared_Infor_sub_;
  // 新增  接受  拼接图的边 维护信息
  rclcpp::Subscription<tare_planner::msg::MergerGraphEdge>::SharedPtr MergerGraphEdge_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_waypoint_sub_;

  // ROS publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_full_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr old_global_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr to_nearest_global_subspace_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_tsp_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr exploration_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr exploration_finish_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr runtime_breakdown_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr runtime_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr momentum_activation_count_pub_;

  //新增 发布  世界网格 状态
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr NewGridWorldCell_and_status_pub_;
  //新增  发布  求解MTSP问题的子图
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr MTSP_SubGrapher_pub_; // 新修改
  //新增发布  目标点 到 farplanning
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_to_farplaner_pub_;
  //新增发布  是否需要   目标 规划  到  farplanning
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr need_farplaner_planning_pub_;

  //新增 move_base
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr move_base_goal_pub_;

  //新增发布   是否返回起点
   rclcpp::Publisher<tare_planner::msg::ReturnHome>::SharedPtr is_returning_home_pub_;
   //新增接受  是否返回起点


  // 新增  客户端和服务端
  // 客户端请求    路径
  // 服务端  使用   keyposegrapher   计算  路径
  rclcpp::Service<tare_planner::srv::RequestPath>::SharedPtr request_path_server_;   //服务端
  // 客户端容器      存多个客户端   计算得到了就使用
  std::vector<rclcpp::Client<tare_planner::srv::RequestPath>::SharedPtr> request_path_client_list_;

  // Debug
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pointcloud_manager_neighbor_cells_origin_pub_;

  //新增  发布  邻接网格子图
  rclcpp::Publisher<tare_planner::msg::Subgraph>::SharedPtr current_subgraph_pub_;

  // 新增  SharedMergerInfor  的发布
  rclcpp::Publisher<tare_planner::msg::SharedMergerInfor>::SharedPtr SharedMergerInfor_pub_;

  // 新增 MergerGraphEdge   的 发布     拼接图 的共享信息
  rclcpp::Publisher<tare_planner::msg::MergerGraphEdge>::SharedPtr MergerGraphEdge_pub_;

  void ReadParameters();
  void InitializeData();

  // Callback functions
  void ExplorationStartCallback(const std_msgs::msg::Bool::ConstSharedPtr start_msg);
  void StateEstimationCallback(const nav_msgs::msg::Odometry::ConstSharedPtr state_estimation_msg);
  void RegisteredScanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_cloud_msg);
  void TerrainMapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_msg);
  void TerrainMapExtCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_cloud_large_msg);
  void CoverageBoundaryCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygon_msg);
  void ViewPointBoundaryCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygon_msg);
  void NogoBoundaryCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygon_msg);

  //新增网格世界单元更新回调
  void NewGridWorldCellStatusCallback(const std_msgs::msg::Int32MultiArray::ConstSharedPtr NewGridWorldCellStatus_msg);
  //新增  MTSP子图  回调
  void MTSP_SubGrapherCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr MTSP_SubGrapher_msg);
  // 新增回调 网格子图回调
  void SubgraphCallback(const tare_planner::msg::Subgraph::ConstSharedPtr Subgraph_msg);
  //新增  回调  返回起点
  void is_returning_home_Callback(const tare_planner::msg::ReturnHome::ConstSharedPtr is_returning_home_msg);
  //新增回调
  void SharedMergerInforCallback(const tare_planner::msg::SharedMergerInfor::ConstSharedPtr SharedMergerInfor_msg);
  // 新增  回调
  void MergerGraphEdgeCallback(const tare_planner::msg::MergerGraphEdge::ConstSharedPtr MergerGraphEdge_msg);



  void JoystickCallback(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg);
  void ResetWaypointCallback(const std_msgs::msg::Empty::ConstSharedPtr empty_msg);

  void SendInitialWaypoint();
  void UpdateKeyposeGraph();
  int UpdateViewPoints();
  void UpdateViewPointCoverage();
  void UpdateRobotViewPointCoverage();
  void UpdateCoveredAreas(int &uncovered_point_num, int &uncovered_frontier_point_num);
  void UpdateVisitedPositions();
  void UpdateGlobalRepresentation();
  void GlobalPlanning(std::vector<int> &global_cell_tsp_order, exploration_path_ns::ExplorationPath &global_path);

  //新增  全局规划
  void GlobalPlanning_pro(std::vector<int>& global_cell_tsp_order, exploration_path_ns::ExplorationPath& global_path,bool& is_global_tsp);
  //新增  全局规划
  void GlobalPlanning_max(std::vector<int>& global_cell_tsp_order, exploration_path_ns::ExplorationPath& global_path,bool& is_global_tsp);
  //新增全局规划  第三代
  void GlobalPlanning_grid_graph(std::vector<int>& global_cell_tsp_order, exploration_path_ns::ExplorationPath& global_path,bool& is_global_tsp);
  // 新增  全局规划第4代  使用 merger graph  代替keypose_graph  的  功能
  void GlobalPlanning_grid_merger_graph(std::vector<int>& global_cell_tsp_order, exploration_path_ns::ExplorationPath& global_path,bool& is_global_tsp);

  void PublishGlobalPlanningVisualization(const exploration_path_ns::ExplorationPath &global_path, const exploration_path_ns::ExplorationPath &local_path);
  void LocalPlanning(int uncovered_point_num, int uncovered_frontier_point_num, const exploration_path_ns::ExplorationPath &global_path, exploration_path_ns::ExplorationPath &local_path);

  //添加 修改的局部路径规划    只需要当前位置  和前视点（上一次目标点） 和
  void LocalPlanning_pro(int uncovered_point_num, int uncovered_frontier_point_num, const exploration_path_ns::ExplorationPath& global_path, exploration_path_ns::ExplorationPath& local_path);
  
  // 添加  优化版本的  局部规划
  void LocalPlanning_opt(int uncovered_point_num, int uncovered_frontier_point_num, const std::vector<exploration_path_ns::ExplorationPath> &near_localcoverage_subgrid_paths, exploration_path_ns::ExplorationPath &local_path);

  //添加 获取靠近局部规划框的  探索子网格到机器人的路径
  void Get_subgrid_paths(std::vector<exploration_path_ns::ExplorationPath>& near_localcoverage_subgrid_paths,
  std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);

  void PublishLocalPlanningVisualization(const exploration_path_ns::ExplorationPath &local_path);

  exploration_path_ns::ExplorationPath ConcatenateGlobalLocalPath(const exploration_path_ns::ExplorationPath &global_path, const exploration_path_ns::ExplorationPath &local_path);

  //添加  更改全局状态
  void GlobalStatuUpdate();

  // 添加 新增   merger_graph_  的更新
  void UpdateMergerGraph();

  //新添修改
  exploration_path_ns::ExplorationPath SortLocalPath(const exploration_path_ns::ExplorationPath& local_path);

  void PublishRuntime();
  double GetRobotToHomeDistance();
  void PublishExplorationState();
  void PublishWaypoint();
  bool
  GetLookAheadPoint(const exploration_path_ns::ExplorationPath &local_path, const exploration_path_ns::ExplorationPath &global_path, Eigen::Vector3d &lookahead_point);

  //新增    GetLookAheadPoint_Globalpath  从全局路径得到前视点  发布
  bool GetLookAheadPoint_Globalpath(const exploration_path_ns::ExplorationPath& global_path, Eigen::Vector3d& lookahead_point);

  bool GetLookAheadPoint_Globalpath_max(const exploration_path_ns::ExplorationPath& global_path, Eigen::Vector3d& lookahead_point);
  //新增 GetLookAheadPoint_Localpath     从局部路径得到前视点 发布
  bool GetLookAheadPoint_Localpath(const exploration_path_ns::ExplorationPath& local_path, Eigen::Vector3d& lookahead_point);
  // 新增  服务
  bool request_path_server(const std::shared_ptr<tare_planner::srv::RequestPath::Request> request,  std::shared_ptr<tare_planner::srv::RequestPath::Response> response);
  void  GetLookAheadPoint_Globalpath_Far(const exploration_path_ns::ExplorationPath &global_path, Eigen::Vector3d &lookahead_point);

  // 新增 是否需要做 全局规划
  bool Is_Global_Planner(const exploration_path_ns::ExplorationPath& local_path);

  void PrintExplorationStatus(std::string status, bool clear_last_line = true);
  void CountDirectionChange();

  // 添加函数  判定局部路径规划   类型
  bool GetLocalPlanningType(const exploration_path_ns::ExplorationPath& global_path);

  // 添加函数    将本地的 SharedMergerInfor 信息加入   全局  Shared_Infor_map_ 中
  void AddLocalSharedMergerInfor(tare_planner::msg::SharedMergerInfor &Shared_Infor);

  // 添加函数  从单向的  全局导航路径 和  全局局部高分辨率路径   中选择 目标点 机器人 
  void GetLookAheadPoint_Globalpath_GlobalLocalpath(const exploration_path_ns::ExplorationPath& global_path,const exploration_path_ns::ExplorationPath& global_local_path,Eigen::Vector3d &lookahead_point);

};

} // namespace sensor_coverage_planner_3d_ns
