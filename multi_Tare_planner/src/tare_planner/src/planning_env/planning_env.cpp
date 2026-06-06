/**
 * @file planning_env.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the world representation using point clouds
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <planning_env/planning_env.h>
#include <viewpoint_manager/viewpoint_manager.h>

#include <algorithm>
#include <utility>

#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <opencv2/opencv.hpp>

namespace planning_env_ns
{
void PlanningEnvParameters::ReadParameters(rclcpp::Node::SharedPtr nh)
{
  nh->get_parameter("kSurfaceCloudDwzLeafSize", kSurfaceCloudDwzLeafSize);
  nh->get_parameter("kCollisionCloudDwzLeafSize", kCollisionCloudDwzLeafSize);
  nh->get_parameter("keypose_graph/kAddEdgeCollisionCheckRadius", kKeyposeGraphCollisionCheckRadius);
  nh->get_parameter("keypose_graph/kAddEdgeCollisionCheckPointNumThr", kKeyposeGraphCollisionCheckPointNumThr);
  nh->get_parameter("kKeyposeCloudStackNum", kKeyposeCloudStackNum);
  nh->get_parameter("kPointCloudRowNum", kPointCloudRowNum);
  nh->get_parameter("kPointCloudColNum", kPointCloudColNum);
  nh->get_parameter("kPointCloudLevelNum", kPointCloudLevelNum);
  nh->get_parameter("kMaxCellPointNum", kMaxCellPointNum);
  nh->get_parameter("kPointCloudCellSize", kPointCloudCellSize);
  nh->get_parameter("kPointCloudCellHeight", kPointCloudCellHeight);
  nh->get_parameter("kPointCloudManagerNeighborCellNum", kPointCloudManagerNeighborCellNum);
  nh->get_parameter("kCoverCloudZSqueezeRatio", kCoverCloudZSqueezeRatio);
  nh->get_parameter("kUseFrontier", kUseFrontier);
  nh->get_parameter("kFrontierClusterTolerance", kFrontierClusterTolerance);
  nh->get_parameter("kFrontierClusterMinSize", kFrontierClusterMinSize);
  nh->get_parameter("kUseCoverageBoundaryOnFrontier", kUseCoverageBoundaryOnFrontier);
  nh->get_parameter("kUseCoverageBoundaryOnObjectSurface", kUseCoverageBoundaryOnObjectSurface);
  int viewpoint_number = nh->get_parameter("viewpoint_manager/number_x").as_int(); // 视点数量
  double viewpoint_resolution = nh->get_parameter("viewpoint_manager/resolution_x").as_double();
  double sensor_range = nh->get_parameter("kSensorRange").as_double();

  double local_planning_horizon_half_size = viewpoint_number * viewpoint_resolution / 2;
  kExtractFrontierRange.x() = local_planning_horizon_half_size + sensor_range * 2;
  kExtractFrontierRange.y() = local_planning_horizon_half_size + sensor_range * 2;
  kExtractFrontierRange.z() = 2;
}

PlanningEnv::PlanningEnv(rclcpp::Node::SharedPtr nh, std::string world_frame_id)
  : keypose_cloud_count_(0)
  , vertical_surface_extractor_()
  , vertical_frontier_extractor_()
  , robot_position_update_(false)
  , nh_(nh)
  , world_frame_id_(std::move(world_frame_id))
{
  parameters_.ReadParameters(nh);

  keypose_cloud_stack_.resize(parameters_.kKeyposeCloudStackNum);
  for (int i = 0; i < keypose_cloud_stack_.size(); i++)
  {
    keypose_cloud_stack_[i].reset(new pcl::PointCloud<PlannerCloudPointType>());
  }

  vertical_surface_cloud_stack_.resize(parameters_.kKeyposeCloudStackNum);
  for (int i = 0; i < vertical_surface_cloud_stack_.size(); i++)
  {
    vertical_surface_cloud_stack_[i].reset(new pcl::PointCloud<PlannerCloudPointType>());
  }
  keypose_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "keypose_cloud", world_frame_id); //关键位姿点云
  stacked_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "stacked_cloud", world_frame_id); //堆叠点云
  stacked_vertical_surface_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
      nh, "stacked_vertical_surface_cloud", world_frame_id); //堆叠垂直表面点云

  stacked_vertical_surface_cloud_kdtree_ =
      pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr(new pcl::KdTreeFLANN<PlannerCloudPointType>()); //堆叠垂直表面点云kd树
  vertical_surface_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "coverage_cloud", world_frame_id); //覆盖点云

  diff_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "diff_cloud", world_frame_id); //差分点云

  collision_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>); //碰撞点云

  terrain_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_cloud", world_frame_id); //地形点云

  planner_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "planner_cloud", world_frame_id); //规划点云

  pointcloud_manager_ = std::make_shared<pointcloud_manager_ns::PointCloudManager>(
      parameters_.kPointCloudRowNum, parameters_.kPointCloudColNum, parameters_.kPointCloudLevelNum,
      parameters_.kMaxCellPointNum, parameters_.kPointCloudCellSize, parameters_.kPointCloudCellHeight,
      parameters_.kPointCloudManagerNeighborCellNum);
  pointcloud_manager_->SetCloudDwzFilterLeafSize() = parameters_.kSurfaceCloudDwzLeafSize;

  rolling_occupancy_grid_ = std::make_shared<rolling_occupancy_grid_ns::RollingOccupancyGrid>(nh);

  squeezed_planner_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
      nh, "squeezed_planner_cloud", world_frame_id); // 压缩规划点云

  squeezed_planner_cloud_kdtree_ =
      pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr(new pcl::KdTreeFLANN<PlannerCloudPointType>()); //压缩规划点云kd树

  uncovered_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uncovered_cloud", world_frame_id); // 未覆盖点云
  uncovered_frontier_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uncovered_frontier_cloud", world_frame_id); // 未覆盖前沿点云
  frontier_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "frontier_cloud", world_frame_id); // 前沿点云
  filtered_frontier_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "filtered_frontier_cloud", world_frame_id); // 过滤前沿点云
    explored_boundary_marker_ = std::make_shared<misc_utils_ns::Marker>(nh, "explored_boundary_marker", world_frame_id_);
    explored_boundary_marker_->SetType(visualization_msgs::msg::Marker::LINE_LIST);
    explored_boundary_marker_->SetScale(0.05, 0.0, 0.0);
    explored_boundary_marker_->SetColorRGBA(0.15, 0.75, 1.0, 0.95);
    explored_boundary_history_marker_ = std::make_shared<misc_utils_ns::Marker>(nh, "explored_boundary_history_marker", world_frame_id_);
    explored_boundary_history_marker_->SetType(visualization_msgs::msg::Marker::LINE_LIST);
    explored_boundary_history_marker_->SetScale(0.03, 0.0, 0.0);
    explored_boundary_history_marker_->SetColorRGBA(0.15, 0.75, 1.0, 0.30);
    explored_boundary_pub_ = nh->create_publisher<geometry_msgs::msg::PolygonStamped>("explored_boundary", 2);
    explored_boundary_msg_.header.frame_id = world_frame_id_;
  occupied_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "occupied_cloud", world_frame_id); // 占用点云
  free_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "free_cloud", world_frame_id); // 空闲点云
  unknown_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "unknown_cloud", world_frame_id);// 未知点云

  rolling_occupancy_grid_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "rolling_occupancy_grid_cloud", world_frame_id); // 滚动占用栅格点云

  rolling_frontier_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolling_frontier_cloud", world_frame_id); // 滚动前沿点云

  rolling_filtered_frontier_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>( 
      nh, "rolling_filtered_frontier_cloud", world_frame_id); // 滚动过滤前沿点云

  rolled_in_occupancy_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolled_in_occupancy_cloud", world_frame_id); // 滚入占用点云
  rolled_out_occupancy_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolled_out_occupancy_cloud", world_frame_id); // 滚出占用点云

  pointcloud_manager_occupancy_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "pointcloud_manager_occupancy_cloud_", world_frame_id); // 点云管理器占用点云

  kdtree_frontier_cloud_ = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>); // 前沿点云kdtree
  kdtree_rolling_frontier_cloud_ = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>); // 滚动前沿点云kdtree

  // Todo: parameterize
  vertical_surface_extractor_.SetRadiusThreshold(0.2);
  vertical_surface_extractor_.SetZDiffMax(2.0);
  vertical_surface_extractor_.SetZDiffMin(parameters_.kSurfaceCloudDwzLeafSize);
  vertical_frontier_extractor_.SetNeighborThreshold(2);

  Eigen::Vector3d rolling_occupancy_grid_resolution = rolling_occupancy_grid_->GetResolution();
  double vertical_frontier_neighbor_search_radius =
      std::max(rolling_occupancy_grid_resolution.x(), rolling_occupancy_grid_resolution.y());
  vertical_frontier_neighbor_search_radius =
      std::max(vertical_frontier_neighbor_search_radius, rolling_occupancy_grid_resolution.z());
  vertical_frontier_extractor_.SetRadiusThreshold(vertical_frontier_neighbor_search_radius);
  double z_diff_max = vertical_frontier_neighbor_search_radius * 5;
  double z_diff_min = vertical_frontier_neighbor_search_radius;
  vertical_frontier_extractor_.SetZDiffMax(z_diff_max);
  vertical_frontier_extractor_.SetZDiffMin(z_diff_min);
  vertical_frontier_extractor_.SetNeighborThreshold(2);
}

// 更新碰撞点云
void PlanningEnv::UpdateCollisionCloud()
{
  collision_cloud_->clear();
  for (int i = 0; i < parameters_.kKeyposeCloudStackNum; i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>()); // 临时点云
    pcl::copyPointCloud<PlannerCloudPointType, pcl::PointXYZI>(*vertical_surface_cloud_stack_[i], *cloud_tmp); // 转换点云类型
    *(collision_cloud_) += *cloud_tmp; // 累加到碰撞点云
  }
  // 下采样碰撞点云
  collision_cloud_downsizer_.Downsize(collision_cloud_, parameters_.kCollisionCloudDwzLeafSize,
                                      parameters_.kCollisionCloudDwzLeafSize, parameters_.kCollisionCloudDwzLeafSize);
}

// 更新前向点云 提取、过滤、聚类前沿点云
void PlanningEnv::UpdateFrontiers()
{
  if (parameters_.kUseFrontier) // 使用前沿点
  {
    prev_robot_position_ = robot_position_;

    // Extract frontiers 提取前沿点云 边界点
    rolling_occupancy_grid_->GetFrontier(frontier_cloud_->cloud_, robot_position_, parameters_.kExtractFrontierRange);

    if (!frontier_cloud_->cloud_->points.empty())
    {
      if (parameters_.kUseCoverageBoundaryOnFrontier)
      {
        GetCoverageCloudWithinBoundary<pcl::PointXYZI>(frontier_cloud_->cloud_);
      }
      // 过滤出垂直方向的前沿点（如墙面边缘，排除地面/天花板）
      vertical_frontier_extractor_.ExtractVerticalSurface<pcl::PointXYZI, pcl::PointXYZI>(
          frontier_cloud_->cloud_, filtered_frontier_cloud_->cloud_);
    }

    // Cluster frontiers 聚类前向点云
    if (!filtered_frontier_cloud_->cloud_->points.empty())
    {
      kdtree_frontier_cloud_->setInputCloud(filtered_frontier_cloud_->cloud_);
      std::vector<pcl::PointIndices> cluster_indices;     // 聚类索引
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec; // 利用欧几里德聚类方法进行聚类
      ec.setClusterTolerance(parameters_.kFrontierClusterTolerance);
      ec.setMinClusterSize(1);     // 设置聚类的最小尺寸
      ec.setMaxClusterSize(10000); // 设置聚类的最大尺寸
      ec.setSearchMethod(kdtree_frontier_cloud_);         // 设置搜索方法为kdtree
      ec.setInputCloud(filtered_frontier_cloud_->cloud_); // 设置输入点云
      ec.extract(cluster_indices); // 执行聚类，得到聚类索引

      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      int cluster_count = 0;
      for (int i = 0; i < cluster_indices.size(); i++)
      {
        if (cluster_indices[i].indices.size() < parameters_.kFrontierClusterMinSize)
        {
          continue;
        }
        for (int j = 0; j < cluster_indices[i].indices.size(); j++)
        {
          int point_ind = cluster_indices[i].indices[j];
          filtered_frontier_cloud_->cloud_->points[point_ind].intensity = cluster_count;
          inliers->indices.push_back(point_ind);
        }
        cluster_count++;
      }
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      extract.setInputCloud(filtered_frontier_cloud_->cloud_);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*(filtered_frontier_cloud_->cloud_));
      filtered_frontier_cloud_->Publish();
    }
  }
}

// 更新地形点云
void PlanningEnv::UpdateTerrainCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  if (cloud->points.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("standalone_logger"), "Terrain cloud empty");
  }
  else
  {
    terrain_cloud_->cloud_ = cloud;
  }
}

// 碰撞检测
bool PlanningEnv::InCollision(double x, double y, double z) const
{
  if (stacked_cloud_->cloud_->points.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("standalone_logger"),
                "PlanningEnv::InCollision(): collision cloud empty, not checking collision"); // 碰撞点云为空，不进行碰撞检测
    return false;
  }
  PlannerCloudPointType check_point;
  check_point.x = x;
  check_point.y = y;
  check_point.z = z;
  std::vector<int> neighbor_indices;
  std::vector<float> neighbor_sqdist;
  stacked_vertical_surface_cloud_kdtree_->radiusSearch(check_point, parameters_.kKeyposeGraphCollisionCheckRadius,
                                                       neighbor_indices, neighbor_sqdist);
  if (neighbor_indices.size() > parameters_.kKeyposeGraphCollisionCheckPointNumThr)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// 更新覆盖区域
void PlanningEnv::UpdateCoveredArea(const lidar_model_ns::LiDARModel& robot_viewpoint,
                                    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
{
  if (planner_cloud_->cloud_->points.empty())
  {
    std::cout << "Planning cloud empty, cannot update covered area" << std::endl;
    return;
  }

  geometry_msgs::msg::Point robot_position = robot_viewpoint.getPosition();
  double sensor_range = viewpoint_manager->GetSensorRange();
  double coverage_occlusion_thr = viewpoint_manager->GetCoverageOcclusionThr();
  double coverage_dilation_radius = viewpoint_manager->GetCoverageDilationRadius();
  std::vector<int> covered_point_indices;
  double vertical_fov_ratio = 0.3;  // bigger fov than viewpoints
  double diff_z_max = sensor_range * vertical_fov_ratio;
  double xy_dist_threshold = 3 * (parameters_.kSurfaceCloudDwzLeafSize / 2) / 0.3;
  double z_diff_threshold = 3 * parameters_.kSurfaceCloudDwzLeafSize;
  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      planner_cloud_->cloud_->points[i].g = 255;
      continue;
    }
    if (std::abs(point.z - robot_position.z) < diff_z_max)
    {
      if (misc_utils_ns::InFOVSimple(Eigen::Vector3d(point.x, point.y, point.z),
                                     Eigen::Vector3d(robot_position.x, robot_position.y, robot_position.z),
                                     vertical_fov_ratio, sensor_range, xy_dist_threshold, z_diff_threshold))
      {
        if (robot_viewpoint.CheckVisibility<PlannerCloudPointType>(point, coverage_occlusion_thr))
        {
          planner_cloud_->cloud_->points[i].g = 255;
          covered_point_indices.push_back(i);
          continue;
        }
      }
    }
    // mark covered by visited viewpoints
    for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
    {
      if (viewpoint_manager->ViewPointVisited(viewpoint_ind))
      {
        if (viewpoint_manager->VisibleByViewPoint<PlannerCloudPointType>(point, viewpoint_ind))
        {
          planner_cloud_->cloud_->points[i].g = 255;
          covered_point_indices.push_back(i);
          break;
        }
      }
    }
  }

  // Dilate the covered area
  squeezed_planner_cloud_->cloud_->clear();
  for (const auto& point : planner_cloud_->cloud_->points)
  {
    PlannerCloudPointType squeezed_point = point;
    squeezed_point.z = point.z / parameters_.kCoverCloudZSqueezeRatio;
    squeezed_planner_cloud_->cloud_->points.push_back(squeezed_point);
  }
  squeezed_planner_cloud_kdtree_->setInputCloud(squeezed_planner_cloud_->cloud_);

  for (const auto& ind : covered_point_indices)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[ind];
    std::vector<int> nearby_indices;
    std::vector<float> nearby_sqdist;
    squeezed_planner_cloud_kdtree_->radiusSearch(point, coverage_dilation_radius, nearby_indices, nearby_sqdist);
    if (!nearby_indices.empty())
    {
      for (const auto& idx : nearby_indices)
      {
        MY_ASSERT(idx >= 0 && idx < planner_cloud_->cloud_->points.size());
        planner_cloud_->cloud_->points[idx].g = 255;
      }
    }
  }

  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      int cloud_idx = 0;
      int cloud_point_idx = 0;
      pointcloud_manager_->GetCloudPointIndex(i, cloud_idx, cloud_point_idx);
      pointcloud_manager_->UpdateCoveredCloudPoints(cloud_idx, cloud_point_idx);
    }
  }

  UpdateExploredBoundary();
}

void PlanningEnv::GetUncoveredArea(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                   int& uncovered_point_num, int& uncovered_frontier_point_num)
{
  // Clear viewpoint covered point list 清除视点覆盖点列表
  for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
  {
    viewpoint_manager->ResetViewPointCoveredPointList(viewpoint_ind);
  }

  // Get uncovered points from planner cloud 得到未覆盖点
  uncovered_cloud_->cloud_->clear();
  uncovered_frontier_cloud_->cloud_->clear();
  uncovered_point_num = 0;
  uncovered_frontier_point_num = 0;
  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      continue;
    }
    bool observed = false;
    for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
    {
      if (!viewpoint_manager->ViewPointVisited(viewpoint_ind))
      {
        if (viewpoint_manager->VisibleByViewPoint<PlannerCloudPointType>(point, viewpoint_ind))
        {
          viewpoint_manager->AddUncoveredPoint(viewpoint_ind, uncovered_point_num);
          observed = true;
        }
      }
    }
    if (observed)
    {
      pcl::PointXYZI uncovered_point;
      uncovered_point.x = point.x;
      uncovered_point.y = point.y;
      uncovered_point.z = point.z;
      uncovered_point.intensity = i;
      uncovered_cloud_->cloud_->points.push_back(uncovered_point);
      uncovered_point_num++;
    }
  }

  // Check uncovered frontiers 检查未覆盖前沿点
  if (parameters_.kUseFrontier)
  {
    for (int i = 0; i < filtered_frontier_cloud_->cloud_->points.size(); i++)
    {
      pcl::PointXYZI point = filtered_frontier_cloud_->cloud_->points[i];
      bool observed = false;
      for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
      {
        if (!viewpoint_manager->ViewPointVisited(viewpoint_ind))
        {
          if (viewpoint_manager->VisibleByViewPoint<pcl::PointXYZI>(point, viewpoint_ind))
          {
            viewpoint_manager->AddUncoveredFrontierPoint(viewpoint_ind, uncovered_frontier_point_num);
            observed = true;
          }
        }
      }
      if (observed)
      {
        pcl::PointXYZI uncovered_frontier_point;
        uncovered_frontier_point.x = point.x;
        uncovered_frontier_point.y = point.y;
        uncovered_frontier_point.z = point.z;
        uncovered_frontier_point.intensity = i;
        uncovered_frontier_cloud_->cloud_->points.push_back(uncovered_frontier_point);
        uncovered_frontier_point_num++;
      }
    }
  }
}

void PlanningEnv::GetVisualizationPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud)
{
  pointcloud_manager_->GetVisualizationPointCloud(vis_cloud);
}

void PlanningEnv::PublishStackedCloud()
{
  stacked_cloud_->Publish();
}

void PlanningEnv::PublishUncoveredCloud()
{
  uncovered_cloud_->Publish();
}

void PlanningEnv::PublishUncoveredFrontierCloud()
{
  uncovered_frontier_cloud_->Publish();
}

void PlanningEnv::UpdateExploredBoundary()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr explored_points(new pcl::PointCloud<pcl::PointXYZI>());
  explored_points->clear();
  for (const auto& point : planner_cloud_->cloud_->points)
  {
    if (point.g <= 0)
    {
      continue;
    }

    pcl::PointXYZI explored_point;
    explored_point.x = point.x;
    explored_point.y = point.y;
    explored_point.z = 0.0;
    explored_point.intensity = point.g;
    explored_points->points.push_back(explored_point);
  }

  if (explored_points->points.size() < 3)
  {
    return;
  }

  explored_boundary_downsizer_.Downsize(explored_points, parameters_.kSurfaceCloudDwzLeafSize,
                                       parameters_.kSurfaceCloudDwzLeafSize, parameters_.kSurfaceCloudDwzLeafSize);

  if (explored_points->points.size() < 3)
  {
    return;
  }

  // Use image-projection based contour extraction (inspired by far_planner::ContourDetector)
  // Build a 2D occupancy image from explored points, find contours with OpenCV,
  // convert contours back to world coordinates and publish as polygon/line markers.

  std::vector<geometry_msgs::msg::Point> current_line_points;
  std::vector<geometry_msgs::msg::Point> history_line_points;
  current_line_points.reserve(explored_points->points.size() * 2);
  history_line_points.reserve(explored_points->points.size() * 2);
  std::vector<geometry_msgs::msg::Point32> current_polygon_points;

  // Compute grid bounds
  float min_x = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  for (const auto &p : explored_points->points) {
    if (p.x < min_x) min_x = p.x;
    if (p.x > max_x) max_x = p.x;
    if (p.y < min_y) min_y = p.y;
    if (p.y > max_y) max_y = p.y;
  }
  // Add small margin
  const float margin = std::max(0.5f, static_cast<float>(parameters_.kSurfaceCloudDwzLeafSize * 2.0));
  min_x -= margin; max_x += margin; min_y -= margin; max_y += margin;

  const float voxel = static_cast<float>(std::max(0.01, parameters_.kSurfaceCloudDwzLeafSize));
  int cols = static_cast<int>(std::ceil((max_x - min_x) / voxel));
  int rows = static_cast<int>(std::ceil((max_y - min_y) / voxel));
  // clamp image size to avoid pathological cases
  const int MAX_DIM = 2048;
  if (cols <= 0 || rows <= 0 || cols > MAX_DIM || rows > MAX_DIM) {
    return;
  }

  cv::Mat img = cv::Mat::zeros(rows, cols, CV_32FC1);
  for (const auto &pt : explored_points->points) {
    int c = static_cast<int>((pt.x - min_x) / (max_x - min_x + 1e-6) * (cols - 1));
    int r = static_cast<int>((max_y - pt.y) / (max_y - min_y + 1e-6) * (rows - 1));
    if (r >= 0 && r < rows && c >= 0 && c < cols) {
      img.at<float>(r, c) += 1.0f;
    }
  }

  // Threshold to binary image
  cv::Mat bin;
  cv::threshold(img, bin, 0.5, 255, cv::THRESH_BINARY);
  bin.convertTo(bin, CV_8UC1);

  // Optionally resize/blur to remove noise (kept small)
  // Find contours
  std::vector<std::vector<cv::Point>> raw_contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(bin, raw_contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (raw_contours.empty()) {
    return;
  }

  // Convert contours to world coords and build polygon/line lists
  for (const auto &cont : raw_contours) {
    if (cont.size() < 3) continue;
    // Optionally simplify contour
    std::vector<cv::Point> approx;
    double eps = std::max(1.0, voxel * 1.0) ;
    cv::approxPolyDP(cont, approx, eps, true);
    if (approx.size() < 3) continue;

    std::vector<geometry_msgs::msg::Point32> poly32;
    poly32.reserve(approx.size());
    for (size_t i = 0; i < approx.size(); ++i) {
      int r = approx[i].y;
      int c = approx[i].x;
      geometry_msgs::msg::Point32 wp;
      wp.x = min_x + (static_cast<float>(c) + 0.5f) * voxel;
      wp.y = max_y - (static_cast<float>(r) + 0.5f) * voxel;
      wp.z = 0.0f;
      poly32.push_back(wp);
    }

    // build lines
    for (size_t i = 0; i < poly32.size(); ++i) {
      const auto &s = poly32[i];
      const auto &e = poly32[(i + 1) % poly32.size()];
      geometry_msgs::msg::Point sp, ep;
      sp.x = s.x; sp.y = s.y; sp.z = 0.0;
      ep.x = e.x; ep.y = e.y; ep.z = 0.0;
      current_line_points.push_back(sp);
      current_line_points.push_back(ep);
      history_line_points.push_back(sp);
      history_line_points.push_back(ep);
    }

    // if we don't have a polygon yet, take the largest contour as current polygon
    if (poly32.size() > current_polygon_points.size()) {
      current_polygon_points = poly32;
    }
  }

  auto append_hull = [&](const pcl::PointCloud<pcl::PointXYZI>::Ptr& hull_points, bool use_as_current_polygon) {
    if (hull_points->points.size() < 3)
    {
      return;
    }

    if (use_as_current_polygon)
    {
      current_polygon_points.clear();
      current_polygon_points.reserve(hull_points->points.size());
      for (const auto& point : hull_points->points)
      {
        geometry_msgs::msg::Point32 boundary_point32;
        boundary_point32.x = static_cast<float>(point.x);
        boundary_point32.y = static_cast<float>(point.y);
        boundary_point32.z = 0.0F;
        current_polygon_points.push_back(boundary_point32);
      }
    }

    for (std::size_t i = 0; i < hull_points->points.size(); ++i)
    {
      const auto& start = hull_points->points[i];
      const auto& end = hull_points->points[(i + 1) % hull_points->points.size()];

      geometry_msgs::msg::Point start_point;
      start_point.x = start.x;
      start_point.y = start.y;
      start_point.z = 0.0;
      geometry_msgs::msg::Point end_point;
      end_point.x = end.x;
      end_point.y = end.y;
      end_point.z = 0.0;

      current_line_points.push_back(start_point);
      current_line_points.push_back(end_point);
      history_line_points.push_back(start_point);
      history_line_points.push_back(end_point);
    }
  };

  // contours already generated from image projection above (current_polygon_points,
  // current_line_points and history_line_points filled).

  if (current_line_points.empty())
  {
    return;
  }

  explored_boundary_msg_.polygon.points = current_polygon_points;
  explored_boundary_marker_->marker_.points = current_line_points;
  explored_boundary_marker_->SetAction(visualization_msgs::msg::Marker::ADD);

  explored_boundary_history_marker_->marker_.points.insert(explored_boundary_history_marker_->marker_.points.end(),
                                                         history_line_points.begin(), history_line_points.end());
  explored_boundary_history_marker_->SetAction(visualization_msgs::msg::Marker::ADD);

  explored_boundary_marker_->SetAction(visualization_msgs::msg::Marker::ADD);
  misc_utils_ns::Publish(nh_, explored_boundary_pub_, explored_boundary_msg_, world_frame_id_);
  explored_boundary_marker_->Publish();
  explored_boundary_history_marker_->Publish();
}

}  // namespace planning_env_ns