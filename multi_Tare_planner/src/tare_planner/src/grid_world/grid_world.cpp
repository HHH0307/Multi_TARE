#include "../../include/grid_world/grid_world.h"
#include <map>
#include <algorithm>
#include <stack>
#include <utils/misc_utils.h>
#include <viewpoint_manager/viewpoint_manager.h>
#include <merger_graph/merger_graph.h>

namespace grid_world_ns
{
Cell::Cell(double x, double y, double z)
  : in_horizon_(false)
  , robot_position_set_(false)
  , visit_count_(0)
  , keypose_id_(0)
  , path_added_to_keypose_graph_(false)
  , roadmap_connection_point_set_(false)
  , viewpoint_position_(Eigen::Vector3d(x, y, z))
  , roadmap_connection_point_(Eigen::Vector3d(x, y, z))
{
  center_.x = x;
  center_.y = y;
  center_.z = z;

  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;
  status_ = CellStatus::UNSEEN;
  MTSP_graph_index_find_exploring_cell_ = -1; // 新增
}

Cell::Cell(const geometry_msgs::msg::Point& center) : Cell(center.x, center.y, center.z)
{
}

void Cell::Reset()
{
  status_ = CellStatus::UNSEEN;
  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;
  visit_count_ = 0;
  viewpoint_indices_.clear();
  connected_cell_indices_.clear();
  connected_cell_set_.clear();
  keypose_graph_node_indices_.clear();
  MTSP_graph_index_find_exploring_cell_ = -1; // 新增
}

// IsCellConnected now inline in header with O(1) set lookup

std::vector<int> GridWorld::GetCellConnectedCellIndices(int cell_ind)
{
  return subspaces_->GetCell(cell_ind).GetConnectedCellIndices();
}

GridWorld::GridWorld(rclcpp::Node::SharedPtr nh) : initialized_(false), use_keypose_graph_(false)
{
  ReadParameters(nh);
  robot_position_.x = 0.0;
  robot_position_.y = 0.0;
  robot_position_.z = 0.0;

  origin_.x = 0.0;
  origin_.y = 0.0;
  origin_.z = 0.0;

  Eigen::Vector3i grid_size(kRowNum, kColNum, kLevelNum);
  Eigen::Vector3d grid_origin(0.0, 0.0, 0.0);
  Eigen::Vector3d grid_resolution(kCellSize, kCellSize, kCellHeight);
  Cell cell_tmp;
  subspaces_ = std::make_shared<grid_ns::Grid<Cell>>(grid_size, cell_tmp, grid_origin, grid_resolution); 
  subspaces_local_ = std::make_shared<grid_ns::Grid<Cell>>(grid_size, cell_tmp, grid_origin, grid_resolution);  // 新增
  subspaces_world_ = std::make_shared<grid_ns::Grid<Cell>>(grid_size, cell_tmp, grid_origin, grid_resolution);  // 新增
  for (int i = 0; i < subspaces_->GetCellNumber(); ++i)
  {
    subspaces_->GetCell(i) = grid_world_ns::Cell();
    subspaces_local_->GetCell(i) = grid_world_ns::Cell();
    subspaces_world_->GetCell(i) = grid_world_ns::Cell();

  }

  home_position_.x() = 0.0;
  home_position_.y() = 0.0;
  home_position_.z() = 0.0;

  cur_keypose_graph_node_position_.x = 0.0;
  cur_keypose_graph_node_position_.y = 0.0;
  cur_keypose_graph_node_position_.z = 0.0;

  set_home_ = false;
  return_home_ = false;

  cur_robot_cell_ind_ = -1;
  prev_robot_cell_ind_ = -1;
  last_robot_cell_ind_ = -1; // 新增
  robot_statu_ = RobotStatus::Exploring; // 新增
  allocation_strategy_ = "Mdvrp"; // 新增
}

GridWorld::GridWorld(int row_num, int col_num, int level_num, double cell_size, double cell_height, int nearby_grid_num)
  : kRowNum(row_num)
  , kColNum(col_num)
  , kLevelNum(level_num)
  , kCellSize(cell_size)
  , kCellHeight(cell_height)
  , KNearbyGridNum(nearby_grid_num)
  , kMinAddPointNumSmall(60)
  , kMinAddPointNumBig(100)
  , kMinAddFrontierPointNum(30)
  , kCellExploringToCoveredThr(1)
  , kCellCoveredToExploringThr(10)
  , kCellExploringToAlmostCoveredThr(10)
  , kCellAlmostCoveredToExploringThr(20)
  , kCellUnknownToExploringThr(1)
  , cur_keypose_id_(0)
  , cur_keypose_graph_node_ind_(0)
  , cur_robot_cell_ind_(-1)
  , prev_robot_cell_ind_(-1)
  , cur_keypose_(0, 0, 0)
  , initialized_(false)
  , use_keypose_graph_(false)
{
  robot_position_.x = 0.0;
  robot_position_.y = 0.0;
  robot_position_.z = 0.0;

  origin_.x = 0.0;
  origin_.y = 0.0;
  origin_.z = 0.0;

  Eigen::Vector3i grid_size(kRowNum, kColNum, kLevelNum); // (121, 121, 121)
  Eigen::Vector3d grid_origin(0.0, 0.0, 0.0);
  Eigen::Vector3d grid_resolution(kCellSize, kCellSize, kCellHeight);
  Cell cell_tmp;
  subspaces_ = std::make_shared<grid_ns::Grid<Cell>>(grid_size, cell_tmp, grid_origin, grid_resolution);
  subspaces_local_ = std::make_shared<grid_ns::Grid<Cell>>(grid_size, cell_tmp, grid_origin, grid_resolution);
  subspaces_world_ = std::make_shared<grid_ns::Grid<Cell>>(grid_size, cell_tmp, grid_origin, grid_resolution);
  for (int i = 0; i < subspaces_->GetCellNumber(); ++i)
  {
    subspaces_->GetCell(i) = grid_world_ns::Cell();
    subspaces_local_->GetCell(i) = grid_world_ns::Cell();
    subspaces_world_->GetCell(i) = grid_world_ns::Cell();
  }

  home_position_.x() = 0.0;
  home_position_.y() = 0.0;
  home_position_.z() = 0.0;

  cur_keypose_graph_node_position_.x = 0.0;
  cur_keypose_graph_node_position_.y = 0.0;
  cur_keypose_graph_node_position_.z = 0.0;

  set_home_ = false;
  return_home_ = false;

  robot_statu_ = RobotStatus::Exploring; // 新增
  allocation_strategy_ = "Mdvrp"; // 新增
}

void GridWorld::ReadParameters(rclcpp::Node::SharedPtr nh)
{
  nh->get_parameter("kGridWorldXNum", kRowNum); // 网格世界  X的数量 121
  nh->get_parameter("kGridWorldYNum", kColNum); // 网格世界  Y的数量 121
  nh->get_parameter("kGridWorldZNum", kLevelNum); // 网格世界  Z的数量 121
  int viewpoint_number = nh->get_parameter("viewpoint_manager/number_x").as_int();
  double viewpoint_resolution = nh->get_parameter("viewpoint_manager/resolution_x").as_double();
  kCellSize = viewpoint_number * viewpoint_resolution / 5;
  nh->get_parameter("kGridWorldCellHeight", kCellHeight);
  nh->get_parameter("kGridWorldNearbyGridNum", KNearbyGridNum);
  nh->get_parameter("kMinAddPointNumSmall", kMinAddPointNumSmall);
  nh->get_parameter("kMinAddPointNumBig", kMinAddPointNumBig);
  nh->get_parameter("kMinAddFrontierPointNum", kMinAddFrontierPointNum);
  nh->get_parameter("kCellExploringToCoveredThr", kCellExploringToCoveredThr);
  nh->get_parameter("kCellCoveredToExploringThr", kCellCoveredToExploringThr);
  nh->get_parameter("kCellExploringToAlmostCoveredThr", kCellExploringToAlmostCoveredThr);
  nh->get_parameter("kCellAlmostCoveredToExploringThr", kCellAlmostCoveredToExploringThr);
  nh->get_parameter("kCellUnknownToExploringThr", kCellUnknownToExploringThr);

  // 新增
  int local_planning_x_num = nh->get_parameter("viewpoint_manager/number_x").as_int();
  double local_planning_resolution_x = nh->get_parameter("viewpoint_manager/resolution_x").as_double();
  kLocal_planning_radius = local_planning_x_num * local_planning_resolution_x * 0.5;  // 15m
  std::string robot_name = nh->get_parameter("robot_name").as_string();
  cur_robot_id_ = (int)robot_name.back() - (int)('0');
  allocation_strategy_ = nh->get_parameter("allocation_strategy_").as_string();
}

void GridWorld::UpdateNeighborCells(const geometry_msgs::msg::Point& robot_position)
{
  if (!initialized_)
  {
    initialized_ = true;
    is_fuse_ = false; // 新增
    robot_statu_ = RobotStatus::Exploring; // 新增
    Request_robot_id_ = 0; // 新增

    /* 2025.10.21 修改 */
    // origin_.x = robot_position.x - (kCellSize * kRowNum) / 2;
    // origin_.y = robot_position.y - (kCellSize * kColNum) / 2;
    // origin_.z = robot_position.z - (kCellHeight * kLevelNum) / 2;
    origin_.x = 0 - (kCellSize * kRowNum) / 2;
    origin_.y = 0 - (kCellSize * kColNum) / 2;
    origin_.z = 0 - (kCellHeight * kLevelNum) / 2;

    subspaces_->SetOrigin(Eigen::Vector3d(origin_.x, origin_.y, origin_.z));
    subspaces_local_->SetOrigin(Eigen::Vector3d(origin_.x, origin_.y, origin_.z)); // 新增
    subspaces_world_->SetOrigin(Eigen::Vector3d(origin_.x, origin_.y, origin_.z)); // 新增

    int robot_id = GetCellInd(robot_position.x, robot_position.y, robot_position.z);// 新增
    //更新此单元的  状态    为已探索。
    subspaces_->GetCell(robot_id).SetStatus(CellStatus::COVERED); // 新增
    subspaces_local_->GetCell(robot_id).SetStatus(CellStatus::COVERED); // 新增
    subspaces_world_->GetCell(robot_id).SetStatus(CellStatus::COVERED); // 新增

    // Update cell centers
    for (int i = 0; i < kRowNum; i++)
    {
      for (int j = 0; j < kColNum; j++)
      {
        for (int k = 0; k < kLevelNum; k++)
        {
          Eigen::Vector3d subspace_center_position = subspaces_->Sub2Pos(i, j, k);
          geometry_msgs::msg::Point subspace_center_geo_position;
          subspace_center_geo_position.x = subspace_center_position.x();
          subspace_center_geo_position.y = subspace_center_position.y();
          subspace_center_geo_position.z = subspace_center_position.z();
          subspaces_->GetCell(i, j, k).SetPosition(subspace_center_geo_position);
          subspaces_->GetCell(i, j, k).SetRoadmapConnectionPoint(subspace_center_position);
        }
      }
    }
  }

  // Get neighbor cells
  std::vector<int> prev_neighbor_cell_indices = neighbor_cell_indices_;
  neighbor_cell_indices_.clear();
  int N = KNearbyGridNum / 2;
  int M = 1;
  GetNeighborCellIndices(robot_position, Eigen::Vector3i(N, N, M), neighbor_cell_indices_);

  for (const auto& cell_ind : neighbor_cell_indices_)
  {
    if (std::find(prev_neighbor_cell_indices.begin(), prev_neighbor_cell_indices.end(), cell_ind) ==
        prev_neighbor_cell_indices.end())
    {
      subspaces_->GetCell(cell_ind).AddVisitCount();
    }
  }
}

void GridWorld::UpdateRobotPosition(const geometry_msgs::msg::Point& robot_position)
{
  robot_position_ = robot_position;
  int robot_cell_ind = GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
  if (cur_robot_cell_ind_ != robot_cell_ind)
  {
    prev_robot_cell_ind_ = cur_robot_cell_ind_;
    cur_robot_cell_ind_ = robot_cell_ind;
  }
}

void GridWorld::UpdateCellKeyposeGraphNodes(const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  std::vector<int> keypose_graph_connected_node_indices = keypose_graph->GetConnectedGraphNodeIndices();

  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_local_->GetCell(i).GetStatus() == CellStatus::EXPLORING) // 修改
    {
      subspaces_->GetCell(i).ClearGraphNodeIndices();
    }
  }
  for (const auto& node_ind : keypose_graph_connected_node_indices)
  {
    geometry_msgs::msg::Point node_position = keypose_graph->GetNodePosition(node_ind);
    int cell_ind = GetCellInd(node_position.x, node_position.y, node_position.z);
    if (subspaces_->InRange(cell_ind))
    {
      if (subspaces_local_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING) // 修改
      {
        subspaces_->GetCell(cell_ind).AddGraphNode(node_ind);
      }
    }
  }
}

bool GridWorld::AreNeighbors(int cell_ind1, int cell_ind2)
{
  Eigen::Vector3i cell_sub1 = subspaces_->Ind2Sub(cell_ind1);
  Eigen::Vector3i cell_sub2 = subspaces_->Ind2Sub(cell_ind2);
  Eigen::Vector3i diff = cell_sub1 - cell_sub2;
  if (std::abs(diff.x()) + std::abs(diff.y()) + std::abs(diff.z()) == 1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

int GridWorld::GetCellInd(double qx, double qy, double qz)
{
  Eigen::Vector3i sub = subspaces_->Pos2Sub(qx, qy, qz);
  if (subspaces_->InRange(sub))
  {
    return subspaces_->Sub2Ind(sub);
  }
  else
  {
    return -1;
  }
}

void GridWorld::GetCellSub(int& row_idx, int& col_idx, int& level_idx, double qx, double qy, double qz)
{
  Eigen::Vector3i sub = subspaces_->Pos2Sub(qx, qy, qz);
  row_idx = (sub.x() >= 0 && sub.x() < kRowNum) ? sub.x() : -1;
  col_idx = (sub.y() >= 0 && sub.y() < kColNum) ? sub.y() : -1;
  level_idx = (sub.z() >= 0 && sub.z() < kLevelNum) ? sub.z() : -1;
}

Eigen::Vector3i GridWorld::GetCellSub(const Eigen::Vector3d& point)
{
  return subspaces_->Pos2Sub(point);
}

void GridWorld::GetMarker(visualization_msgs::msg::Marker& marker)
{
  marker.points.clear();
  marker.colors.clear();
  marker.scale.x = kCellSize;
  marker.scale.y = kCellSize;
  marker.scale.z = kCellHeight;

  int exploring_count = 0;
  int covered_count = 0;
  int unseen_count = 0;

  bool is_show_global = true; // 新增

  if(is_show_global)
  {
    for (int i = 0; i < kRowNum; i++)
    {
      for (int j = 0; j < kColNum; j++)
      {
        for (int k = 0; k < kLevelNum; k++)
        {
          int cell_ind = subspaces_->Sub2Ind(i, j, k);
          geometry_msgs::msg::Point cell_center = subspaces_->GetCell(cell_ind).GetPosition();
          std_msgs::msg::ColorRGBA color;
          bool add_marker = false;
          if (subspaces_world_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN)
          {
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
            color.a = 0.1;
            unseen_count++;
          }
          else if (subspaces_world_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED)
          {
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
            color.a = 0.2; // 0.1
            covered_count++;
            add_marker = true;
          }
          else if (subspaces_world_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING)
          {
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
            color.a = 0.2; // 0.1
            exploring_count++;
            add_marker = true;
          }
          else if (subspaces_world_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED_BY_OTHERS) // 原 NOGO
          {
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            color.a = 0.2; // 0.1
            add_marker = true;
          }
          else
          {
            color.r = 1.0;  // 0.8
            color.g = 0.0;  // 0.8
            color.b = 0.0;  // 0.8
            color.a = 1.0;  // 0.1
            // add_marker = true;
          }
          if (add_marker)
          {
            marker.colors.push_back(color);
            marker.points.push_back(cell_center);
          }
        }
      }
    }
  }
  else
  {
    for (int i = 0; i < kRowNum; i++)
    {
      for (int j = 0; j < kColNum; j++)
      {
        for (int k = 0; k < kLevelNum; k++)
        {
          int cell_ind = subspaces_->Sub2Ind(i, j, k);
          geometry_msgs::msg::Point cell_center = subspaces_->GetCell(cell_ind).GetPosition();
          std_msgs::msg::ColorRGBA color;
          bool add_marker = false;
          if (subspaces_local_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN)
          {
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
            color.a = 0.1;
            unseen_count++;
          }
          else if (subspaces_local_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED)
          {
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
            color.a = 0.2;  //  0.1
            covered_count++;
            add_marker = true;
          }
          else if (subspaces_local_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING)
          {
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
            color.a = 0.2;
            exploring_count++;
            add_marker = true;
          }
          else if (subspaces_local_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED_BY_OTHERS) // 原 NOGO
          {
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            color.a = 0.2;
            add_marker = true;
          }
          else
          {
            color.r = 1;  // 0.8
            color.g = 0;  // 0.8
            color.b = 0;  // 0.8
            color.a = 1;
            // add_marker = true;
          }
          if (add_marker)
          {
            marker.colors.push_back(color);
            marker.points.push_back(cell_center);
          }
        }
      }
    }
  }
}

void GridWorld::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud)
{
  vis_cloud->points.clear();
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    CellStatus cell_status = subspaces_->GetCell(i).GetStatus();
    if (!subspaces_->GetCell(i).GetConnectedCellIndices().empty())
    {
      pcl::PointXYZI point;
      Eigen::Vector3d position = subspaces_->GetCell(i).GetRoadmapConnectionPoint();
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.intensity = i;
      vis_cloud->points.push_back(point);
    }
  }
}

void GridWorld::AddViewPointToCell(int cell_ind, int viewpoint_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).AddViewPoint(viewpoint_ind);
}

void GridWorld::AddGraphNodeToCell(int cell_ind, int node_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).AddGraphNode(node_ind);
}

void GridWorld::ClearCellViewPointIndices(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).ClearViewPointIndices();
}

std::vector<int> GridWorld::GetCellViewPointIndices(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetViewPointIndices();
}

void GridWorld::GetNeighborCellIndices(const Eigen::Vector3i& center_cell_sub, const Eigen::Vector3i& neighbor_range,
                                       std::vector<int>& neighbor_indices)
{
  int row_idx = 0;
  int col_idx = 0;
  int level_idx = 0;
  for (int i = -neighbor_range.x(); i <= neighbor_range.x(); i++)
  {
    for (int j = -neighbor_range.y(); j <= neighbor_range.y(); j++)
    {
      row_idx = center_cell_sub.x() + i;
      col_idx = center_cell_sub.y() + j;
      for (int k = -neighbor_range.z(); k <= neighbor_range.z(); k++)
      {
        level_idx = center_cell_sub.z() + k;
        Eigen::Vector3i sub(row_idx, col_idx, level_idx);
        if (subspaces_->InRange(sub))
        {
          int ind = subspaces_->Sub2Ind(sub);
          neighbor_indices.push_back(ind); // 新增
        }
      }
    }
  }
}
void GridWorld::GetNeighborCellIndices(const geometry_msgs::msg::Point& position, const Eigen::Vector3i& neighbor_range,
                                       std::vector<int>& neighbor_indices)
{
  Eigen::Vector3i center_cell_sub = GetCellSub(Eigen::Vector3d(position.x, position.y, position.z));

  GetNeighborCellIndices(center_cell_sub, neighbor_range, neighbor_indices);
}

void GridWorld::GetExploringCellIndices(std::vector<int>& exploring_cell_indices)
{
  exploring_cell_indices.clear();
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_local_->GetCell(i).GetStatus() == CellStatus::EXPLORING)
    {
      exploring_cell_indices.push_back(i);
    }
  }
}

void GridWorld::GetExploringCellIndicesWorld(std::vector<int>& exploring_cell_indices_world)
{
  exploring_cell_indices_world.clear();
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_->GetCell(i).GetStatus() == CellStatus::EXPLORING)
    {
      exploring_cell_indices_world.push_back(i);
    }
  }
}

void GridWorld::GetExploringAndCoveredCellIndicesWorld(std::vector<int>& cell_indices_world)
{
  cell_indices_world.clear();
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    CellStatus status = subspaces_->GetCell(i).GetStatus();
    if (status == CellStatus::EXPLORING || status == CellStatus::COVERED)
    {
      cell_indices_world.push_back(i);
    }
  }
}

CellStatus GridWorld::GetCellStatus(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_local_->GetCell(cell_ind).GetStatus();
}
CellStatus GridWorld::GetCellStatus_world(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetStatus();
}

void GridWorld::SetCellStatus(int cell_ind, grid_world_ns::CellStatus status)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).SetStatus(status);
}

geometry_msgs::msg::Point GridWorld::GetCellPosition(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetPosition();
}

// 阶段3.1: 获取 cell 内的 merger_graph 节点索引列表
std::vector<int> GridWorld::GetCellMergerGraphNodeIndices(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetMergerGraphNodeIndices();
}

void GridWorld::SetCellRobotPosition(int cell_ind, const geometry_msgs::msg::Point& robot_position)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).SetRobotPosition(robot_position);
}

geometry_msgs::msg::Point GridWorld::GetCellRobotPosition(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetRobotPosition();
}

void GridWorld::CellAddVisitCount(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).AddVisitCount();
}

int GridWorld::GetCellVisitCount(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetVisitCount();
}

bool GridWorld::IsRobotPositionSet(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).IsRobotPositionSet();
}

void GridWorld::Reset()
{
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    subspaces_->GetCell(i).Reset();
  }
}

int GridWorld::GetCellStatusCount(grid_world_ns::CellStatus status)
{
  int count = 0;
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_local_->GetCell(i).GetStatus() == status)
    {
      count++;
    }
  }
  return count;
}

//新增  得到当前机器人状态
grid_world_ns::RobotStatus GridWorld::GetRobotStatu()
{
  return robot_statu_;
}

//  修改  状态更新 -- 新增
void GridWorld::UpdateCellStatus_(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
{
  int exploring_count = 0;
  int unseen_count = 0;
  int covered_count = 0;
  for (int i = 0; i < subspaces_->GetCellNumber(); ++i)
  {
    if (subspaces_local_->GetCell(i).GetStatus() == CellStatus::EXPLORING)  //探索
    {
      exploring_count++;
    }
    else if (subspaces_local_->GetCell(i).GetStatus() == CellStatus::UNSEEN)  //无法看见
    {
      unseen_count++;
    }
    else if (subspaces_local_->GetCell(i).GetStatus() == CellStatus::COVERED)  //覆盖
    {
      covered_count++;
    }
  }

  for (const auto& cell_ind : neighbor_cell_indices_)  //邻接的单元  ID  根据  机器人位置移动  而  改变的 ID
  {
    subspaces_->GetCell(cell_ind).ClearViewPointIndices();  //清除当前邻接单元 里面的候选视点
                                                            //这是前一次保留的，在下面会更新       由于视点更新
  }
  for (const auto& viewpoint_ind :
       viewpoint_manager->candidate_indices_)  //候选视点 ID    给对应的单元Cell       填充候选视点
  {
    geometry_msgs::msg::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);  //得到候选视点  位置信息
    Eigen::Vector3i sub = subspaces_->Pos2Sub(Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z));  //得到候选视点  在    子空间里的三维坐标
    if (subspaces_->InRange(sub))                                            //视点是否在子空间里面
    {
      int cell_ind = subspaces_->Sub2Ind(sub);  //子空间编码成cellID
      AddViewPointToCell(cell_ind,
                         viewpoint_ind);  //对应的单元添加  候选视点（候选视点即是  不在障碍物里，在视线内，是连通的）
      viewpoint_manager->SetViewPointCellInd(viewpoint_ind, cell_ind);  //更新视点所在单元编号
    }
    else
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("standalone_logger"), "subspace sub out of bound: " << sub.transpose());
    }
  }
  //更新每个邻接网格    的 状态
  for (const auto& cell_ind : neighbor_cell_indices_)
  {
    if (subspaces_local_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED_BY_OTHERS)  
    {
      continue;
    }
    // 邻接 子网格  的状态信息
    int candidate_count = 0;                 //候选视点数量
    int selected_viewpoint_count = 0;        //被选中的视点数量
    int above_big_threshold_count = 0;       // 大于   高阈值 的数量
    int above_small_threshold_count = 0;     // 大于  低阈值的数量
    int above_frontier_threshold_count = 0;  // 大于  边阈值的数量
    int highest_score_viewpoint_ind = -1;    // 最高分  视点  索引
    int highest_score = -1;                  // 最高分
    // 对子网格  编号为   cell_ind  的  处理
    for (const auto& viewpoint_ind : subspaces_->GetCell(cell_ind).GetViewPointIndices())  
    { 
      MY_ASSERT(viewpoint_manager->IsViewPointCandidate(viewpoint_ind));
      candidate_count++;                                        //子网格单元的   候选点数 属性  +1
      if (viewpoint_manager->ViewPointSelected(viewpoint_ind))  //当前是视点    是否是 本次
                                                                //局部规划中被选中的做局部TSP的目标点
      {
        selected_viewpoint_count++;  //子网格 单元 的  被选中点数   属性  +1
      }
      if (viewpoint_manager->ViewPointVisited(viewpoint_ind))  //当前视点被访问过，不做处理
                                                               //即当前视点是处于被其他人探索过的
      {
        continue;  // 若是被访问过  就跳过    被访问的设置  是由前一次    规划后更新的
      }
      int score = viewpoint_manager->GetViewPointCoveredPointNum(viewpoint_ind);  //视点分数   当前视点     能够覆盖
                                                                                  //未探索点的数量
      int frontier_score = viewpoint_manager->GetViewPointCoveredFrontierPointNum(
          viewpoint_ind);         //视点边界分数     当前视点    能够覆盖未探索边界点的数量
      if (score > highest_score)  //获得当前子网格单元内   单个视点的最高分  和对应的  视点索引
      {
        highest_score = score;
        highest_score_viewpoint_ind = viewpoint_ind;
      }
      if (score > kMinAddPointNumSmall)  //  当  视点  分数大于最小添加   40
      {
        above_small_threshold_count++;  //大于  低阈值的数量    计数+1
      }
      if (score > kMinAddPointNumBig)  //分数大于最小添加      最大值  60
      {
        above_big_threshold_count++;  //大于   高阈值数量   计数+1
      }
      if (frontier_score > kMinAddFrontierPointNum)  //边界分数   大于边界点阈值    20
      {
        above_frontier_threshold_count++;  //大于边界阈值  +1
      }
    }
    // Exploring to Covered    探索  转  覆盖       当前    subspaces_local_  的网格  状态  是   探索状态 且满足一定条件
    if (subspaces_local_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING &&
        above_frontier_threshold_count < kCellExploringToCoveredThr &&
        above_small_threshold_count < kCellExploringToCoveredThr && selected_viewpoint_count == 0 &&
        candidate_count > 0) 
    {
      subspaces_local_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);  //满足上述条件   将状态改变为覆盖
      if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN ||
          subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING)
      {
        subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);  
        subspaces_world_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);  
      }
    }
    // Covered to Exploring   覆盖  转  探索
    else if (subspaces_local_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED &&
             (above_big_threshold_count >= kCellCoveredToExploringThr ||
              above_frontier_threshold_count >= kCellCoveredToExploringThr))
    {  
      subspaces_local_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING);
      if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN)
      {
        subspaces_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING); 
        subspaces_world_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING);
      }
      almost_covered_cell_indices_.push_back(cell_ind);  
    }
    // Exploring to Almost covered  
    else if (subspaces_local_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING &&
             selected_viewpoint_count == 0 && candidate_count > 0) 
    {
      almost_covered_cell_indices_.push_back(cell_ind);
    }
    else if (subspaces_local_->GetCell(cell_ind).GetStatus() != CellStatus::COVERED &&
             selected_viewpoint_count > 0)
    {
      subspaces_local_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING); 
      if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN)
      {
        subspaces_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING); 
        subspaces_world_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING); 
      }
      almost_covered_cell_indices_.erase(
          std::remove(almost_covered_cell_indices_.begin(), almost_covered_cell_indices_.end(), cell_ind),
          almost_covered_cell_indices_.end());
    }
    else if (subspaces_local_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING &&
             candidate_count == 0)  
    {
      // First visit   第一次访问
      if (subspaces_->GetCell(cell_ind).GetVisitCount() == 1 &&
          subspaces_->GetCell(cell_ind).GetGraphNodeIndices().empty())
      {
        subspaces_local_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
        if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN ||
            subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING)
        {
          subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED); 
          subspaces_world_->GetCell(cell_ind).SetStatus(CellStatus::COVERED); 
        }
      }
      else
      {
        geometry_msgs::msg::Point cell_position = subspaces_->GetCell(cell_ind).GetPosition();
        double xy_dist_to_robot =
            misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(cell_position, robot_position_);
        double z_dist_to_robot = std::abs(cell_position.z - robot_position_.z);  
        if (xy_dist_to_robot < kCellSize && z_dist_to_robot < kCellHeight * 0.8) 
        {
          subspaces_local_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
          if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN ||
              subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING)
          {
            subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
            subspaces_world_->GetCell(cell_ind).SetStatus(CellStatus::COVERED); 
          }
        }
      }
    }
    if (subspaces_local_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING && candidate_count > 0)
    {
      subspaces_->GetCell(cell_ind).SetRobotPosition(robot_position_);
      subspaces_->GetCell(cell_ind).SetKeyposeID(cur_keypose_id_);
    }
  }
  for (const auto& cell_ind : almost_covered_cell_indices_) 
  {
    if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), cell_ind) ==
        neighbor_cell_indices_.end())
    {
      subspaces_local_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
      if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN ||
          subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING)
      {
        subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
        subspaces_world_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
      }
      almost_covered_cell_indices_.erase(
          std::remove(almost_covered_cell_indices_.begin(), almost_covered_cell_indices_.end(), cell_ind),
          almost_covered_cell_indices_.end());
    }
  }

  last_robot_cell_ind_ = cur_robot_cell_ind_;
  if (IndInBound(last_robot_cell_ind_))
  {
    if (subspaces_->GetCell(last_robot_cell_ind_).GetStatus() == CellStatus::UNSEEN ||
        subspaces_->GetCell(last_robot_cell_ind_).GetStatus() == CellStatus::EXPLORING)
    {
      subspaces_->GetCell(last_robot_cell_ind_).SetStatus(CellStatus::COVERED);
    }
    if (subspaces_local_->GetCell(last_robot_cell_ind_).GetStatus() == CellStatus::UNSEEN ||
        subspaces_local_->GetCell(last_robot_cell_ind_).GetStatus() == CellStatus::EXPLORING)
    {
      subspaces_local_->GetCell(last_robot_cell_ind_).SetStatus(CellStatus::COVERED);
    }
    if (subspaces_world_->GetCell(last_robot_cell_ind_).GetStatus() == CellStatus::UNSEEN ||
        subspaces_world_->GetCell(last_robot_cell_ind_).GetStatus() == CellStatus::EXPLORING)
    {
      subspaces_world_->GetCell(last_robot_cell_ind_).SetStatus(CellStatus::COVERED); 
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int GridWorld::GetCellKeyposeID(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetKeyposeID();
}

void GridWorld::GetCellViewPointPositions(std::vector<Eigen::Vector3d>& viewpoint_positions)
{
  viewpoint_positions.clear();
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_->GetCell(i).GetStatus() != grid_world_ns::CellStatus::EXPLORING)
    {
      continue;
    }
    if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), i) == neighbor_cell_indices_.end())
    {
      viewpoint_positions.push_back(subspaces_->GetCell(i).GetViewPointPosition());
    }
  }
}

void GridWorld::AddPathsInBetweenCells(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                       const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  // Determine the connection point in each cell
  for (int i = 0; i < neighbor_cell_indices_.size(); i++)
  {
    int cell_ind = neighbor_cell_indices_[i];
    if (subspaces_->GetCell(cell_ind).IsRoadmapConnectionPointSet())
    {
      if (viewpoint_manager->InLocalPlanningHorizon(subspaces_->GetCell(cell_ind).GetRoadmapConnectionPoint()) &&
          !viewpoint_manager->InCollision(subspaces_->GetCell(cell_ind).GetRoadmapConnectionPoint()))
      {
        continue;
      }
      else
      {
        subspaces_->GetCell(cell_ind).ClearConnectedCellIndices();
      }
    }

    std::vector<int> candidate_viewpoint_indices = subspaces_->GetCell(cell_ind).GetViewPointIndices();
    if (!candidate_viewpoint_indices.empty())
    {
      double min_dist = DBL_MAX;
      double min_dist_viewpoint_ind = candidate_viewpoint_indices.front();
      for (const auto& viewpoint_ind : candidate_viewpoint_indices)
      {
        geometry_msgs::msg::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);
        double dist_to_cell_center = misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
            viewpoint_position, subspaces_->GetCell(cell_ind).GetPosition());
        if (dist_to_cell_center < min_dist)
        {
          min_dist = dist_to_cell_center;
          min_dist_viewpoint_ind = viewpoint_ind;
        }
      }
      geometry_msgs::msg::Point min_dist_viewpoint_position =
          viewpoint_manager->GetViewPointPosition(min_dist_viewpoint_ind);
      subspaces_->GetCell(cell_ind).SetRoadmapConnectionPoint(
          Eigen::Vector3d(min_dist_viewpoint_position.x, min_dist_viewpoint_position.y, min_dist_viewpoint_position.z));
      subspaces_->GetCell(cell_ind).SetRoadmapConnectionPointSet(true);
    }
  }

  for (int i = 0; i < neighbor_cell_indices_.size(); i++)
  {
    int from_cell_ind = neighbor_cell_indices_[i];
    int viewpoint_num = subspaces_->GetCell(from_cell_ind).GetViewPointIndices().size();
    if (viewpoint_num == 0)
    {
      continue;
    }
    std::vector<int> from_cell_connected_cell_indices = subspaces_->GetCell(from_cell_ind).GetConnectedCellIndices();
    Eigen::Vector3d from_cell_roadmap_connection_position =
        subspaces_->GetCell(from_cell_ind).GetRoadmapConnectionPoint();
    if (!viewpoint_manager->InLocalPlanningHorizon(from_cell_roadmap_connection_position))
    {
      continue;
    }
    // Eigen::Vector3i from_cell_sub = ind2sub(from_cell_ind);
    Eigen::Vector3i from_cell_sub = subspaces_->Ind2Sub(from_cell_ind);
    std::vector<int> nearby_cell_indices;
    for (int x = -1; x <= 1; x++)
    {
      for (int y = -1; y <= 1; y++)
      {
        for (int z = -1; z <= 1; z++)
        {
          if (std::abs(x) + std::abs(y) + std::abs(z) == 1)
          {
            Eigen::Vector3i neighbor_sub = from_cell_sub + Eigen::Vector3i(x, y, z);
            // if (SubInBound(neighbor_sub))
            if (subspaces_->InRange(neighbor_sub))
            {
              // int neighbor_ind = sub2ind(neighbor_sub);
              int neighbor_ind = subspaces_->Sub2Ind(neighbor_sub);
              nearby_cell_indices.push_back(neighbor_ind);
            }
          }
        }
      }
    }

    for (int j = 0; j < nearby_cell_indices.size(); j++)
    {
      int to_cell_ind = nearby_cell_indices[j];
      // Just for debug
      if (!AreNeighbors(from_cell_ind, to_cell_ind))
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("standalone_logger"),
                            "Cell " << from_cell_ind << " and " << to_cell_ind << " are not neighbors");
      }
      if (subspaces_->GetCell(to_cell_ind).GetViewPointIndices().empty())
      {
        continue;
      }
      std::vector<int> to_cell_connected_cell_indices = subspaces_->GetCell(to_cell_ind).GetConnectedCellIndices();
      Eigen::Vector3d to_cell_roadmap_connection_position =
          subspaces_->GetCell(to_cell_ind).GetRoadmapConnectionPoint();
      if (!viewpoint_manager->InLocalPlanningHorizon(to_cell_roadmap_connection_position))
      {
        continue;
      }

      // TODO: change to: if there is already a direct keypose graph connection then continue
      bool connected_in_keypose_graph = HasDirectKeyposeGraphConnection(
          keypose_graph, from_cell_roadmap_connection_position, to_cell_roadmap_connection_position);

      bool forward_connected =
          subspaces_->GetCell(from_cell_ind).IsCellConnected(to_cell_ind);
      bool backward_connected =
          subspaces_->GetCell(to_cell_ind).IsCellConnected(from_cell_ind);

      if (connected_in_keypose_graph)
      {
        continue;
      }

      nav_msgs::msg::Path path_in_between = viewpoint_manager->GetViewPointShortestPath(
          from_cell_roadmap_connection_position, to_cell_roadmap_connection_position);

      if (PathValid(path_in_between, from_cell_ind, to_cell_ind))
      {
        path_in_between = misc_utils_ns::SimplifyPath(path_in_between);
        for (auto& pose : path_in_between.poses)
        {
          pose.pose.orientation.w = -1;
        }
        // Add the path
        // std::cout << "Adding path between " << from_cell_ind << " " << to_cell_ind << std::endl;
        // to_connect_cell_paths_.push_back(path_in_between);
        keypose_graph->AddPath(path_in_between);
        bool connected = HasDirectKeyposeGraphConnection(keypose_graph, from_cell_roadmap_connection_position,
                                                         to_cell_roadmap_connection_position);
        if (!connected)
        {
          // Reset both cells' roadmap connection points
          // std::cout << "Resetting both cells connection points" << std::endl;
          subspaces_->GetCell(from_cell_ind).SetRoadmapConnectionPointSet(false);
          subspaces_->GetCell(to_cell_ind).SetRoadmapConnectionPointSet(false);
          subspaces_->GetCell(from_cell_ind).ClearConnectedCellIndices();
          subspaces_->GetCell(to_cell_ind).ClearConnectedCellIndices();
          //  添加删除某个点的函数
          subspaces_->GetCell(from_cell_ind).DelOneCellLongTermConnectedCellIndices(to_cell_ind);
          subspaces_->GetCell(to_cell_ind).DelOneCellLongTermConnectedCellIndices(from_cell_ind);
          continue;
        }
        else
        {
          subspaces_->GetCell(from_cell_ind).AddConnectedCell(to_cell_ind);
          subspaces_->GetCell(to_cell_ind).AddConnectedCell(from_cell_ind);
          //添加长期  连接
          subspaces_->GetCell(from_cell_ind).AddLongTermConnectedCell(to_cell_ind);
          subspaces_->GetCell(to_cell_ind).AddLongTermConnectedCell(from_cell_ind);
        }
      }
    }
  }
}

bool GridWorld::PathValid(const nav_msgs::msg::Path& path, int from_cell_ind, int to_cell_ind)
{
  if (path.poses.size() >= 2)
  {
    for (const auto& pose : path.poses)
    {
      int cell_ind = GetCellInd(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
      if (cell_ind != from_cell_ind && cell_ind != to_cell_ind)
      {
        return false;
      }
    }
    return true;
  }
  else
  {
    return false;
  }
}

bool GridWorld::HasDirectKeyposeGraphConnection(const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                                const Eigen::Vector3d& start_position,
                                                const Eigen::Vector3d& goal_position)
{
  if (!keypose_graph->HasNode(start_position) || !keypose_graph->HasNode(goal_position))
  {
    return false;
  }

  // Search a path connecting start_position and goal_position with a max path length constraint
  geometry_msgs::msg::Point geo_start_position;
  geo_start_position.x = start_position.x();
  geo_start_position.y = start_position.y();
  geo_start_position.z = start_position.z();

  geometry_msgs::msg::Point geo_goal_position;
  geo_goal_position.x = goal_position.x();
  geo_goal_position.y = goal_position.y();
  geo_goal_position.z = goal_position.z();

  double max_path_length = kCellSize * 2;
  nav_msgs::msg::Path path;
  bool found_path = keypose_graph->GetShortestPathWithMaxLength(geo_start_position, geo_goal_position, max_path_length, false, path);
  return found_path;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  UNSEEN = 0,  //看不见单元   编号为0
  EXPLORING = 1,  //探索单元   编号为1
  COVERED = 2,    //覆盖单元   编号为2
  COVERED_BY_OTHERS = 3,    //被其他覆盖   编号为3    （这个的作用是用来互定位的？）
  NOGO = 4   //无法通行  编号为4
*/
void GridWorld::GetUpdateNeighbor_NewGridWorldCellStatus_(std::vector<int>& UpdateNeighbor_GridWorldCellStatus_code_)
{
  //得到更新邻接单元ID集
  std::vector<int> a;
  UpdateNeighbor_GridWorldCellStatus_code_.clear();
  //编码  邻接单元ID  及 其状态
  //    编码   2个为一组   ，前面是编码号，后面  是状态 号
  for (int i = 0; i < neighbor_cell_indices_.size(); i++)  // neighbor_cell_indices_  对于当前位置的 邻接单元
  {
    int cell_ind = neighbor_cell_indices_[i];  //得到邻接单元ID]
    CellStatus statu;
    statu = GetCellStatus(cell_ind);  //编码本地状态？   编码
    if (statu == CellStatus::UNSEEN)
    {
      UpdateNeighbor_GridWorldCellStatus_code_.push_back(cell_ind);
      UpdateNeighbor_GridWorldCellStatus_code_.push_back(0);
    }
    else if (statu == CellStatus::EXPLORING)
    {
      UpdateNeighbor_GridWorldCellStatus_code_.push_back(cell_ind);
      UpdateNeighbor_GridWorldCellStatus_code_.push_back(1);
    }
    else if (statu == CellStatus::COVERED)
    {
      UpdateNeighbor_GridWorldCellStatus_code_.push_back(cell_ind);
      UpdateNeighbor_GridWorldCellStatus_code_.push_back(2);
    }
    else if (statu == CellStatus::COVERED_BY_OTHERS)
    {
      UpdateNeighbor_GridWorldCellStatus_code_.push_back(cell_ind);
      UpdateNeighbor_GridWorldCellStatus_code_.push_back(3);
    }
    else if (statu == CellStatus::NOGO)
    {
      UpdateNeighbor_GridWorldCellStatus_code_.push_back(cell_ind);
      UpdateNeighbor_GridWorldCellStatus_code_.push_back(4);
    }
  }
}

void GridWorld::UpdateGridWorldCellFromOtherRobots(std::map<int, int>& Update_Grid_World_ID_and_Statu_)
{
  const bool debug = true;
  int count_unseen = 0;
  int count_exploring = 0;
  int count_covered = 0;
  int count_covered_by_others = 0;
  int count_nogo = 0;

  //从获得的网格世界 单元  ID和状态  更新到   subspaces_  里面去
  //只同步更新被探索点
  for (auto iter = Update_Grid_World_ID_and_Statu_.begin(); iter != Update_Grid_World_ID_and_Statu_.end(); ++iter)
  {
    if (debug)
    {
      switch (iter->second)
      {
        case 0:
          count_unseen++;
          break;
        case 1:
          count_exploring++;
          break;
        case 2:
          count_covered++;
          break;
        case 3:
          count_covered_by_others++;
          break;
        case 4:
          count_nogo++;
          break;
        default:
          break;
      }
    }
    if (iter->second == 0)
    {
      // subspaces_->GetCell(iter->first).SetStatus(CellStatus::UNSEEN);    //不做变化
    }
    else if (iter->second == 1)  // 传入来的  是探索标记
    {
      if (subspaces_world_->GetCell(iter->first).GetStatus() == CellStatus::UNSEEN)  //若全局是  UNSEEN  则变为  探索
      {
        subspaces_world_->GetCell(iter->first).SetStatus(CellStatus::EXPLORING);  //全局探索点不能反复    只有UNSEE  和
                                                                                  // exploring    能转变过去
      }
    }
    else if (iter->second == 2)  //其他传来是  覆盖      //传入是覆盖  若当前是探索则不变
    {
      if (subspaces_->GetCell(iter->first).GetStatus() == CellStatus::EXPLORING)  //如果本地计算   是探索
                                                                                  //，而被其他传来覆盖
      {
        subspaces_->GetCell(iter->first).SetStatus(CellStatus::COVERED_BY_OTHERS);
        subspaces_world_->GetCell(iter->first).SetStatus(CellStatus::COVERED_BY_OTHERS);
        subspaces_local_->GetCell(iter->first).SetStatus(CellStatus::COVERED_BY_OTHERS);
      }
      else if (subspaces_->GetCell(iter->first).GetStatus() == CellStatus::COVERED)
      {
        continue;  //不做处理
      }
      else  //其余状态则，改为被其他覆盖
      {
        subspaces_->GetCell(iter->first).SetStatus(CellStatus::COVERED_BY_OTHERS);
        subspaces_world_->GetCell(iter->first).SetStatus(CellStatus::COVERED_BY_OTHERS);
        subspaces_local_->GetCell(iter->first).SetStatus(CellStatus::COVERED_BY_OTHERS);
      }
    }
    else if (iter->second == 3)  //其他传来      是 被其他覆盖   若是   本地是探索     则添加到本地探索点
    {
      //查询本地是什么状态
      if (subspaces_->GetCell(iter->first).GetStatus() == CellStatus::COVERED)  //若是覆盖则是自己覆盖的不变
      {
        continue;
      }
      else if (subspaces_->GetCell(iter->first).GetStatus() == CellStatus::EXPLORING)
      {
        subspaces_->GetCell(iter->first).SetStatus(CellStatus::COVERED_BY_OTHERS);
        subspaces_world_->GetCell(iter->first).SetStatus(CellStatus::COVERED_BY_OTHERS);
        subspaces_local_->GetCell(iter->first).SetStatus(CellStatus::COVERED_BY_OTHERS);
      }
      else  //若是其他则改成被其他覆盖
      {
        subspaces_->GetCell(iter->first).SetStatus(CellStatus::COVERED_BY_OTHERS);
        subspaces_world_->GetCell(iter->first).SetStatus(CellStatus::COVERED_BY_OTHERS);
        subspaces_local_->GetCell(iter->first).SetStatus(CellStatus::COVERED_BY_OTHERS);
      }
    }
    else if (iter->second == 4)
    {
      // subspaces_->GetCell(iter->first).SetStatus(CellStatus::NOGO);    //别人不能通行   不代表其他方向不能通行
      // ，这个判定是困难的
    }
  }
  // if (debug)
  // {
  //   std::cout << "[GW][Share] UpdateGridWorldCellFromOtherRobots: total=" << Update_Grid_World_ID_and_Statu_.size()
  //             << " unseen=" << count_unseen << " exploring=" << count_exploring << " covered=" << count_covered
  //             << " covered_by_others=" << count_covered_by_others << " nogo=" << count_nogo << std::endl;
  // }
  Update_Grid_World_ID_and_Statu_.clear();  //更新一次就清空  重新接受
}

//
double GridWorld::GetShortestPath_Fuse_Grapher(fuse_grapher& Fuse_Graoher_, int start_id, int target_id)
{
  if (Fuse_Graoher_.fuse_nodes_.size() < 2)
  {
    return misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
        Fuse_Graoher_.fuse_nodes_[start_id].position_, Fuse_Graoher_.fuse_nodes_[target_id].position_);
  }
  int from_idx = start_id;
  int to_idx = target_id;
  double min_dist_to_start = DBL_MAX;
  double min_dist_to_target = DBL_MAX;
  std::vector<geometry_msgs::msg::Point> node_positions;
  for (int i = 0; i < Fuse_Graoher_.fuse_nodes_.size(); i++)
  {
    node_positions.push_back(Fuse_Graoher_.fuse_nodes_[i].position_);
  }
  bool get_path = false;  //不要路径。
  std::vector<int> path_indices;
  double shortest_dist = misc_utils_ns::AStarSearch(Fuse_Graoher_.graph_, Fuse_Graoher_.dist_, node_positions, from_idx,
                                                    to_idx, get_path, path_indices);
  return shortest_dist;
}

//判断局部路径规划  类型      第一种 是探索逻辑
//，选取未去过的点做目标点，第二种是达到目标逻辑（本地探索方向探索完毕，远点存在未探索的目标点，需要调度这个闲置的机器人去对应目标点，探索。）
bool GridWorld::GetLocalPlanningType(const exploration_path_ns::ExplorationPath& global_path)
{
  //   global_path  有目标都是  3个    单独一个目标的时候   就当作到达目标点前进
  if (is_fuse_)  //个数等于3  即是做MTSP     若是做TSP    则等于2  或者很大
  {
    int first_Cell_id =
        subspaces_->Pos2Ind(global_path.nodes_[1].position_);  //全局规划结束      第一个目标点  是外部未探索点
    geometry_msgs::msg::Point robot_position;
    robot_position.x = global_path.nodes_[0].position_[0];
    robot_position.y = global_path.nodes_[0].position_[1];
    robot_position.z = global_path.nodes_[0].position_[2];
    geometry_msgs::msg::Point first_global;
    first_global.x = global_path.nodes_[1].position_[0];
    first_global.y = global_path.nodes_[1].position_[1];
    first_global.z = global_path.nodes_[1].position_[2];
    //第一个目标点和当前距离  较远
    double distance_ = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(robot_position, first_global);

    //判定  从到达目标逻辑切换到探索逻辑
    if (distance_ > 0.1 * kCellSize)  //融合成功   有了目标到达     且    目标距离本地较远  （  0.3个单元长度）做
                                      //到达目标点逻辑
    {
      return false;
    }
    else
    {
      //以到达目标点逻辑  切换到 探索逻辑，  修改   global_path   去掉目标点，  变成 返回   home  的路径

      is_fuse_ = false;
      robot_statu_ = RobotStatus::Exploring;
      return true;
    }
  }

  return true;  // true  代表做探索逻辑
}


void MTSP_grid_graph::GetConnectedNodeIndices(int query_ind, std::vector<int>& connected_node_indices,
                                              std::vector<bool> constraints)  //从一个点入手
                                                                              //，返回得到所有想连接的点集（可传递）
                                                                              //与关键位姿结点相连接的结点  ID
{
  if (MTSP_graph_nodes_.size() != constraints.size())
  {
    // ROS_ERROR("KeyposeGraph::GetConnectedNodeIndices: constraints size not equal to node size");
    return;
  }
  if (query_ind < 0 || query_ind >= MTSP_graph_nodes_.size())
  {
    // ROS_ERROR_STREAM("KeyposeGraph::GetConnectedNodeIndices: query_ind: " << query_ind << " out of range: [0, "
    //                                                                       << MTSP_graph_nodes_.size() << "]");
    return;
  }
  connected_node_indices.clear();
  std::vector<bool> visited(MTSP_graph_nodes_.size(), false);
  std::stack<int> dfs_stack;  //这个第一个是  当前传入的关键位姿ID
  dfs_stack.push(query_ind);
  while (!dfs_stack.empty())  //不为空
  {
    int current_ind = dfs_stack.top();
    connected_node_indices.push_back(current_ind);  //将当前位姿ID   传入
    dfs_stack.pop();
    if (!visited[current_ind])  //若是这个ID  没有访问过
    {
      visited[current_ind] = true;  //加入访问
    }
    for (int i = 0; i < graph_[current_ind].size(); i++)  //对图中的所有结点进行循环
    {
      //与传入ID  相连接的  图中的结点 ID      graph_  存储了邻接信息，连通就在graph_  上有邻接的
      // graph_  图的更新 根据局部规划框范围更新
      int neighbor_ind = graph_[current_ind][i];
      if (!visited[neighbor_ind] && constraints[neighbor_ind])
      {
        dfs_stack.push(neighbor_ind);
      }
    }
  }
}

void MTSP_grid_graph::GetClosestNodeIndAndDistance(const geometry_msgs::msg::Point& point, int& node_ind, double& dist)
{
  node_ind = -1;
  dist = DBL_MAX;
  if (nodes_cloud_->points.empty())
  {
    node_ind = -1;
    dist = DBL_MAX;
    return;
  }
  pcl::PointXYZI search_point;
  search_point.x = point.x;
  search_point.y = point.y;
  search_point.z = point.z;
  std::vector<int> nearest_neighbor_node_indices(1);
  std::vector<float> nearest_neighbor_squared_dist(1);
  kdtree_nodes_->nearestKSearch(search_point, 1, nearest_neighbor_node_indices, nearest_neighbor_squared_dist);
  if (!nearest_neighbor_node_indices.empty() && nearest_neighbor_node_indices.front() >= 0 &&
      nearest_neighbor_node_indices.front() < nodes_cloud_->points.size())
  {
    node_ind = static_cast<int>(nodes_cloud_->points[nearest_neighbor_node_indices.front()].intensity);
    dist = sqrt(nearest_neighbor_squared_dist.front());
  }
  else
  {
    PCL_WARN_STREAM("KeyposeGraph::GetClosestNodeIndAndDistance: search for nearest neighbor failed with "
                    << nodes_cloud_->points.size() << " nodes.");
    if (!nearest_neighbor_node_indices.empty())
    {
      PCL_WARN_STREAM("Nearest neighbor node Ind: " << nearest_neighbor_node_indices.front());
    }
    for (int i = 0; i < MTSP_graph_nodes_.size(); i++)
    {
      geometry_msgs::msg::Point node_position = MTSP_graph_nodes_[i].position_;
      double dist_to_query =
          misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(point, node_position);
      if (dist_to_query < dist)
      {
        dist = dist_to_query;
        node_ind = i;
      }
    }
  }
}

//添加函数
void MTSP_grid_graph::GetClosestConnectedNodeIndAndDistance(const geometry_msgs::msg::Point& point, int& node_ind,
                                                            double& dist)
{
  node_ind = -1;
  dist = DBL_MAX;
  if (nodes_cloud_->points.empty())
  {
    node_ind = -1;
    dist = DBL_MAX;
    return;
  }
  pcl::PointXYZI search_point;
  search_point.x = point.x;
  search_point.y = point.y;
  search_point.z = point.z;
  std::vector<int> nearest_neighbor_node_indices(1);
  std::vector<float> nearest_neighbor_squared_dist(1);
  kdtree_connected_nodes_->nearestKSearch(search_point, 1, nearest_neighbor_node_indices,
                                          nearest_neighbor_squared_dist);
  if (!nearest_neighbor_node_indices.empty() && nearest_neighbor_node_indices.front() >= 0 &&
      nearest_neighbor_node_indices.front() < connected_nodes_cloud_->points.size())
  {
    node_ind = static_cast<int>(connected_nodes_cloud_->points[nearest_neighbor_node_indices.front()].intensity);
    dist = sqrt(nearest_neighbor_squared_dist.front());
  }
  else
  {
    PCL_WARN_STREAM("KeyposeGraph::GetClosestNodeIndAndDistance: search for nearest neighbor failed with "
                    << connected_nodes_cloud_->points.size() << " nodes.");
    if (!nearest_neighbor_node_indices.empty())
    {
      PCL_WARN_STREAM("Nearest neighbor node Ind: " << nearest_neighbor_node_indices.front());
    }
    for (int i = 0; i < MTSP_graph_nodes_.size(); i++)
    {
      geometry_msgs::msg::Point node_position = MTSP_graph_nodes_[i].position_;
      double dist_to_query =
          misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(point, node_position);
      if (dist_to_query < dist)
      {
        dist = dist_to_query;
        node_ind = i;
      }
    }
  }
}

void MTSP_grid_graph::UpdateNodes()
{
  nodes_cloud_->clear();
  for (int i = 0; i < MTSP_graph_nodes_.size(); i++)
  {
    pcl::PointXYZI point;
    point.x = MTSP_graph_nodes_[i].position_.x;
    point.y = MTSP_graph_nodes_[i].position_.y;
    point.z = MTSP_graph_nodes_[i].position_.z;
    point.intensity = i;  //强度就是编号
    nodes_cloud_->points.push_back(point);
  }
  if (!nodes_cloud_->points.empty())
  {
    kdtree_nodes_->setInputCloud(nodes_cloud_);
  }
}

//构造函数
MTSP_grid_graph::MTSP_grid_graph()
{
  kdtree_connected_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  connected_nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  kdtree_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
}

//添加 获取靠近局部规划框的  探索子网格到机器人的路径
void GridWorld::Get_subgrid_paths(std::vector<exploration_path_ns::ExplorationPath>& near_localcoverage_subgrid_paths,
                                  std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  std::vector<int> neighbor_cell_indices_pro;
  std::vector<int> neighbor_cell_indices;
  int N = KNearbyGridNum / 2 + 1;  // 5/2    x,y        本身以KNearbyGridNum / 2  为邻接    +1    为局部规划框外邻接
  int M = 1;                       // z轴1个为邻接
  GetNeighborCellIndices(robot_position_, Eigen::Vector3i(N, N, M), neighbor_cell_indices_pro);
  N -= 1;
  GetNeighborCellIndices(robot_position_, Eigen::Vector3i(N, N, M), neighbor_cell_indices);
  int max_num = std::max(neighbor_cell_indices_pro.size(), neighbor_cell_indices.size());


  std::vector<int> neighbor_cell_indices_left;  //用于计算前后两次邻接体像素网格的编号差异
  misc_utils_ns::SetDifference(neighbor_cell_indices_pro, neighbor_cell_indices,
                               neighbor_cell_indices_left);  //得到不同的编号
  int left_num = neighbor_cell_indices_left.size();

  std::vector<int> explore_cell_indices;
  std::vector<std::pair<double, int>> near_localcoverage_subgrid_paths_map;
  std::vector<exploration_path_ns::ExplorationPath> near_localcoverage_subgrid_paths_data;

  for (int i = 0; i < left_num; i++)
  {
    int cell_id = neighbor_cell_indices_left[i];
    if (subspaces_world_->GetCell(cell_id).GetStatus() == CellStatus::EXPLORING)  
    {
      Eigen::Vector3d connection_point = subspaces_->GetCell(cell_id).GetRoadmapConnectionPoint(); 
      geometry_msgs::msg::Point connection_point_geo;
      connection_point_geo.x = connection_point.x();
      connection_point_geo.y = connection_point.y();
      connection_point_geo.z = connection_point.z();

      bool reachable = false;  //初始定义当前子空间单元不可通行
      if (keypose_graph->IsPositionReachable(connection_point_geo))
      {
        reachable = true;
      }
      else  //当不可通行时寻找此单元其他关键位姿结点 与当前    connection_point_geo   最近的点
      {
        // Check all the keypose graph nodes within this cell to see if there are any connected nodes
        // 检查此单元格中的所有keypose图形节点，以查看是否存在任何连接的节点
        double min_dist = DBL_MAX;
        double min_dist_node_ind = -1;
        for (const auto& node_ind : subspaces_->GetCell(cell_id).GetGraphNodeIndices())
        {
          geometry_msgs::msg::Point node_position = keypose_graph->GetNodePosition(node_ind);  //单元内  存在于图上的结点集
          double dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(node_position,
                                                                                                connection_point_geo);
          if (dist < min_dist)
          {
            min_dist = dist;
            min_dist_node_ind = node_ind;
          }
        }
        if (min_dist_node_ind >= 0 && min_dist_node_ind < keypose_graph->GetNodeNum())
        {
          reachable = true;
          connection_point_geo = keypose_graph->GetNodePosition(min_dist_node_ind);
        }
      }

      // 获得 当前机器人位置    与能到达的   外围圈  之间的路径
      //当判定是相连的  求当前  机器人位置 robot_position_     和  外围探索点   connection_point_geo   之间的距离
      if (reachable)
      {
        //求  在  keypose_graph中  求   robot_position_   与    connection_point_geo  路径   放入
        // near_localcoverage_subgrid_paths  中 参考   5000 多行的  全局求路径
        nav_msgs::msg::Path cur_keypose_path;  //路径
        // keypose_graph->GetShortestPath(cur_position, next_position, true, keypose_path, false);  //得到两个位置的
        // 连接路径       false
        keypose_graph->GetShortestPath(robot_position_, connection_point_geo, true, cur_keypose_path,
                                       true);    //得到两个位置的   连接路径       false
        if (cur_keypose_path.poses.size() >= 2)  //两点之间的路径
        {
          exploration_path_ns::ExplorationPath explore_path;
          double length = misc_utils_ns::GetPathLength(cur_keypose_path);
          for (int j = 1; j < cur_keypose_path.poses.size() - 1; j++)
          {
            geometry_msgs::msg::Point node_position;
            node_position = cur_keypose_path.poses[j].pose.position;
            exploration_path_ns::Node keypose_node(node_position, exploration_path_ns::NodeType::GLOBAL_VIA_POINT);
            keypose_node.keypose_graph_node_ind_ = static_cast<int>(cur_keypose_path.poses[j].pose.orientation.x);
            explore_path.Append(keypose_node);
          }

          near_localcoverage_subgrid_paths_map.push_back(
              std::pair<double, int>(length, near_localcoverage_subgrid_paths_data.size()));
          near_localcoverage_subgrid_paths_data.push_back(explore_path);
          // near_localcoverage_subgrid_paths.push_back(explore_path);
        }
      }
    }
  }
  //    从  中  near_localcoverage_subgrid_paths_map选择最近的两个  即可  按第一个元素从小到大排序
  std::sort(near_localcoverage_subgrid_paths_map.begin(), near_localcoverage_subgrid_paths_map.end());
  for (int i = 0; i < near_localcoverage_subgrid_paths_map.size(); i++)
  {
    if (near_localcoverage_subgrid_paths.size() == 2)  //测试  置为0  快速  应该置为2 的
    {
      break;
    }
    // exploration_path_ns::ExplorationPath this_path = near_localcoverage_subgrid_paths_map[i].second;
    near_localcoverage_subgrid_paths.push_back(
        near_localcoverage_subgrid_paths_data[near_localcoverage_subgrid_paths_map[i].second]);
  }
}

// 添加函数   添加节点给
void GridWorld::AddNode2MergerGraph(
    const std::shared_ptr<merger_graph_ns::MergerGraph>& merger_graph,
    std::unordered_map<int, tare_planner::msg::SharedMergerInfor>& Shared_Infor_map)
{
  // 加点   维护更新     节点  映射   robot_keyposeind_mergerind_;
  for (auto& Shared_msg : Shared_Infor_map)
  {
    //  当前点属于  哪个   robot    id
    int temp_robot_id = Shared_msg.first;
    for (auto& node : Shared_msg.second.new_node_set)
    {
      //  新增点
      int keypose_node_ind_ = node.node_index;
      //  新点  索引
      int new_node_ind = merger_graph->GetNewNodeIndex();  //新增点索引为  第一个现在现存的节点个数
      int cell_ind = GetCellInd(node.node_position.x, node.node_position.y, node.node_position.z);
      std::vector<int> node_indices = subspaces_->GetCell(cell_ind).GetMergerGraphNodeIndices();

      // 对网格  进行标记   用于 后续    拼接不同机器人边     时去重
      // 获取现存的  robot_id
      std::set<int> robot_set = subspaces_->GetCell(cell_ind).GetRobotIdSet();
      if (robot_set.count(temp_robot_id) == 0 && robot_set.size() == 0)
      {
        //第一次在本网格内添加点
        subspaces_->GetCell(cell_ind).SetRobotIdSet().insert(temp_robot_id);  // 插入机器人id
      }
      else if (robot_set.count(temp_robot_id) == 0 && robot_set.size() != 0)
      {
        //网格内存在  其他机器人点 且不存在 当前机器人的点  添加  false  标记
        // 对  robot_set  做循环  插入 与新点的  标记     新插入的就是 false
        for (auto& robotid : robot_set)
        {
          //  对  添加  robotmin_robotmax_isconnected_
          subspaces_->GetCell(cell_ind).InsterData2RMRMIC(temp_robot_id, robotid, false);
        }
        subspaces_->GetCell(cell_ind).SetRobotIdSet().insert(temp_robot_id);  // 插入机器人id
      }

      if (node_indices.size() == 0)
      {
        //加点
        merger_graph->AddNode(node.node_position, new_node_ind, 0, node.is_keypose, temp_robot_id);
        //设置   点  与   merger_graph  的索引转换    边关系 使用     索引连接
        merger_graph->AddMap2robtkeyposeindmergerind(temp_robot_id, keypose_node_ind_, new_node_ind);
        // 在本网格  上  添加    merger  索引
        subspaces_->GetCell(cell_ind).AddNodeIndex2MergerGraphNodeIndices(new_node_ind);
        continue;
      }
      //   排查  是否可以做  融合  非  本地点  做融合
      int min_node_index = node_indices[0];
      double min_node_dist = DBL_MAX;
      for (auto& index : node_indices)
      {
        //  查寻   网格中  距离  当前  点  最近  的 其他机器人 节点   判断  距离  是否小于   最小增加距离   做索引  融合
        if (merger_graph->GetNodeRobotId(index) == temp_robot_id)
        {
          continue;
        }
        geometry_msgs::msg::Point temp_point = merger_graph->GetNodePosition(index);
        double temp_dist =
            misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(node.node_position, temp_point);
        double z_diff_to_target = std::abs(merger_graph->GetNodePosition(min_node_index).z - node.node_position.z);
        if (temp_dist < min_node_dist && z_diff_to_target < 1.5)  //  除了考虑距离以外还要考虑高度
        {
          min_node_dist = temp_dist;
          min_node_index = index;
        }
      }
      //  判定  是否融合  !!!!!!!!   存在其他机器人的点  与  当前点所属机器人  的距离    小于  最小生成点距离/2
      //  这里的  融点不稳妥   没有做碰撞检测  可能出问题
      //  考虑高度  高度差距
      if (min_node_dist < (merger_graph->GetkAddNonKeyposeNodeMinDist() / 2))
      {
        // 融合  不加新点   更改指向的索引   将边的更改  全部指向存在的点上
        merger_graph->AddMap2robtkeyposeindmergerind(temp_robot_id, keypose_node_ind_, min_node_index);
        //记录  融合点个数
        merger_graph->Addfusion_node_num();

        // 两个  不同机器人的点  融合  之后     那么  标记中  这两个  机器人  就融合了
        int robotid = merger_graph->GetNodeRobotId(min_node_index);
        subspaces_->GetCell(cell_ind).InsterData2RMRMIC(temp_robot_id, robotid, true);
        continue;
      }
      // 不融合  加新点
      merger_graph->AddNode(node.node_position, new_node_ind, 0, node.is_keypose, temp_robot_id);
      //设置   点  与   merger_graph  的索引转换    边关系 使用     索引连接
      merger_graph->AddMap2robtkeyposeindmergerind(temp_robot_id, keypose_node_ind_, new_node_ind);
      // 在本网格  上  添加    merger  索引
      subspaces_->GetCell(cell_ind).AddNodeIndex2MergerGraphNodeIndices(new_node_ind);
    }
  }
}

// 添加函数    网格内不同机器人之间添加边  并更新标记
/*
 //流程
  //  查询   邻接单元
  // 对每个  邻接单元    查询的标记  若现存的机器人标记  都是 true  则 不处理
  //  查询 标记为  false  相关的             多个的机器人的节点索引
  //  分类 对多个机器人  的多个节点索引 计算    最接近网格中心的 代表      点
  //  对于  标记为  false   且  存在    代表点  的  两点  做直线连接  碰撞检测   是否连线
  //  连线成功则  则  修改 标记
*/
void GridWorld::AddDiffRobotEdge2MergerGraphOnNeighborCell(
    const std::shared_ptr<merger_graph_ns::MergerGraph>& merger_graph,
    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
    std::vector<std::pair<int, int>>& add_edge)
{
  //流程
  //  查询   邻接单元  neighbor_cell_indices_
  // 对每个  邻接单元    查询的标记  若现存的机器人标记  都是 true  则 不处理
  for (auto& ind : neighbor_cell_indices_)
  {
    // 查询标记
    //  查询 标记为  false  相关的             多个的机器人的节点索引
    std::vector<std::pair<int, int>> need_edge_robotmin_robotmax;
    // 记录需要  查找中心点的标记
    // std::set<int> need_add_edge_robot_id;
    std::map<int, std::vector<int>> robot_id_merger_graph_node_indices;  // 分不同机器人  获取的节点位置。
    for (auto& temp : subspaces_->GetCell(ind).GetRMRMIC())
    {
      int robot_id_min = temp.first;
      for (auto& temp_next : temp.second)
      {
        if (temp_next.second == false)
        {
          int robot_id_max = temp_next.first;
          std::vector<int> temp_empty_min, temp_empty_max;
          robot_id_merger_graph_node_indices.insert(std::pair<int, std::vector<int>>(robot_id_min, temp_empty_min));
          robot_id_merger_graph_node_indices.insert(std::pair<int, std::vector<int>>(robot_id_max, temp_empty_max));
          // need_add_edge_robot_id.insert(robot_id_min);
          // need_add_edge_robot_id.insert(robot_id_max);
          need_edge_robotmin_robotmax.push_back(std::pair<int, int>(robot_id_min, robot_id_max));
        }
      }
    }
    //  获取  需要连接的        最接近中心点的机器人节点  索引
    std::vector<int> merger_graph_node_indices_temp = subspaces_->GetCell(ind).GetMergerGraphNodeIndices();
    // 各个机器人在这个网格内的   merger_graph_ 的节点索引
    for (auto& node_index : merger_graph_node_indices_temp)
    {
      int robotid = merger_graph->GetNodeRobotId(node_index);
      //  当前点 所属机器人 是  没被标记 且 是连通的点
      if (robot_id_merger_graph_node_indices.count(robotid) > 0 &&
          merger_graph->GetNodeIsConnected(node_index) == true)  //存在当前节点对应的机器人 id
      {
        robot_id_merger_graph_node_indices[robotid].push_back(node_index);
      }
    }
    //  求最接近 各个机器人   最接近   cell_center_position  的节点  索引
    // 网格  位置     使用    中心点
    geometry_msgs::msg::Point cell_center_position = subspaces_->GetCell(ind).GetPosition();
    // 各个机器人  最接近  中心点   的  merger_graph  的索引
    std::map<int, int> robot_id_closest_node_index;
    for (auto& temp_node_indices : robot_id_merger_graph_node_indices)
    {
      int robotid = temp_node_indices.first;
      if (temp_node_indices.second.size() == 0)
      {
        continue;
      }
      int min_node_index = temp_node_indices.second[0];
      double min_node_dist = DBL_MAX;
      for (auto& temp_node_index : temp_node_indices.second)
      {
        geometry_msgs::msg::Point node_position = merger_graph->GetNodePosition(temp_node_index);
        double temp_dist =
            misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(node_position, cell_center_position);
        if (temp_dist < min_node_dist)
        {
          min_node_dist = temp_dist;
          min_node_index = temp_node_index;
        }
      }
      //  存在最中心点 加入索引。
      robot_id_closest_node_index[robotid] = min_node_index;
    }
    //  对于  标记为  false   且  存在    代表点  的  两点  做直线连接  碰撞检测   是否连线
    // need_edge_robotmin_robotmax   需要连线 的  边
    for (auto& edge : need_edge_robotmin_robotmax)
    {
      if (robot_id_closest_node_index.count(edge.first) != 0 && robot_id_closest_node_index.count(edge.second) != 0)
      {
        //碰撞检测  添加边    edge_
        int node_index_1 = robot_id_closest_node_index[edge.first];
        int node_index_2 = robot_id_closest_node_index[edge.second];
        //  获得两个点的  位置数据
        geometry_msgs::msg::Point node_position_1 = merger_graph->GetNodePosition(node_index_1);
        geometry_msgs::msg::Point node_position_2 = merger_graph->GetNodePosition(node_index_2);

        // 判定  边的高度  高度  大于  1.5  则不连线
        double z_diff_to_target = std::abs(node_position_1.z - node_position_2.z);
        if (z_diff_to_target > 1.5)  //高度约束  不能飞   高度约束   考虑楼道上下层 和坡道  上下高度 约束
                                     //需要根据不同实际场景 调试  1.5 这个参数在目前 应该是够用的
        {
          continue;
        }
        //使用  局部规划 做碰撞检测  并连线
        Eigen::Vector3d viewpoint_resolution = viewpoint_manager->GetResolution();  //视点分辨率
        double collision_check_resolution =
            std::min(viewpoint_resolution.x(), viewpoint_resolution.y()) / 2;  // 需要除以2 么？
        Eigen::Vector3d start_position = Eigen::Vector3d(node_position_1.x, node_position_1.y, node_position_1.z);
        ;
        Eigen::Vector3d end_position = Eigen::Vector3d(node_position_2.x, node_position_2.y, node_position_2.z);
        std::vector<Eigen::Vector3d> interp_points;
        misc_utils_ns::LinInterpPoints(start_position, end_position, collision_check_resolution,
                                       interp_points);  //两个结点 ， 碰撞检测分辨率， 线性内部点集合 interp_points
        bool is_connected = true;
        for (const auto& collision_check_position : interp_points)  //检测这些线性内部点集合
        {
          int viewpoint_ind = viewpoint_manager->GetViewPointInd(collision_check_position);  //得到  线性内部点 的ID
          if (viewpoint_manager->InRange(viewpoint_ind))  //判断  此ID  是否在范围内
          {
            if (viewpoint_manager->ViewPointInCollision(viewpoint_ind))  // 是碰撞的
            {
              is_connected = false;
              break;
            }
          }
          else  //  在局部规划框 范围外 不能做   碰撞检测    这次   加点取消
          {
            is_connected = false;
            break;
          }
        }
        //  连线  不同机器人的边      成功则    对   node_index_1   node_index_2  添加边  并添加标记  且不存在边
        //  才连接，存在就不连接
        if (is_connected && merger_graph->HasEdgeBetween(node_index_1, node_index_2) == false)
        {
          // 添加边   添加标记
          double temp_dist =
              misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(node_position_1, node_position_2);
          merger_graph->AddEdge(node_index_1, node_index_2, temp_dist);
          add_edge.push_back(std::pair<int, int>(node_index_1, node_index_2));
          //记录同网格添加边个数
          merger_graph->Addsame_subgrid_edge_num();

          //  对  机器人编号  edge.first   edge.second  添加标记
          subspaces_->GetCell(ind).InsterData2RMRMIC(edge.first, edge.second, true);
        }
      }
    }
  }
}

/**************************************************************************************
 *
 * ！！！！！！！！！！！！！！！！！！分界线！！！！！！！！！！！！！！！！！！！
 * 使用  merger graph  做全局规划  替代  keypose graph
 * SolveGlobalMdvrp_merger_graph
 *
 * ************************************************************************************/
exploration_path_ns::ExplorationPath GridWorld::SolveGlobalMdvrp_merger_graph(
    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
    std::vector<int>& ordered_cell_indices, bool& is_global_tsp,
    std::vector<rclcpp::Client<tare_planner::srv::RequestPath>::SharedPtr>& request_path_client_list_, // 替换为ROS2
    std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
    std::shared_ptr<merger_graph_ns::MergerGraph>& merger_graph,
    std::map<int, geometry_msgs::msg::Point>& other_robot_position_map,
    std::shared_ptr<skeleton_graph_ns::SkeletonGraph>& skeleton_graph)
{
  const bool debug = true;

  exploration_path_ns::ExplorationPath global_path;  // 最终发出的路径
  double min_dist_to_robot = DBL_MAX;
  geometry_msgs::msg::Point global_path_robot_position = robot_position_;  //机器人在全局路径上的  位置
  Eigen::Vector3d eigen_robot_position(robot_position_.x, robot_position_.y,
                                       robot_position_.z);  // eigen 库 形式的机器人位置数据
  // Get nearest connected node   获取最近的连接节点
  int closest_node_ind = 0;
  double closest_node_dist = DBL_MAX;
  keypose_graph->GetClosestConnectedNodeIndAndDistance(
      robot_position_, closest_node_ind, closest_node_dist);  //在关键位姿图上找到当前机器人位置最近的结点和距离
  if (closest_node_dist < kCellSize / 2 && closest_node_ind >= 0 &&
      closest_node_ind < keypose_graph->GetNodeNum())  //当找到的结点最小距离小于kCellSize/2 且结点的范围正常
  {
    global_path_robot_position =
        keypose_graph->GetNodePosition(closest_node_ind);  //当前机器人位置就是找到最近结点在关键位姿图上的位置
  }
  else if (cur_keypose_graph_node_ind_ >= 0 && cur_keypose_graph_node_ind_ < keypose_graph->GetNodeNum())
  {
    // 使用最近的keypose节点进行机器人定位
    global_path_robot_position =
        keypose_graph->GetNodePosition(cur_keypose_graph_node_ind_);  //使用最近的keypose节点当作机器人位姿
  }
  else
  {
    // //使用相邻单元路线图连接点确定机器人位置
    for (int i = 0; i < neighbor_cell_indices_.size(); i++)
    {
      int cell_ind = neighbor_cell_indices_[i];
      if (subspaces_->GetCell(cell_ind).IsRoadmapConnectionPointSet())
      {
        Eigen::Vector3d roadmap_connection_point = subspaces_->GetCell(cell_ind).GetRoadmapConnectionPoint();
        if (viewpoint_manager->InLocalPlanningHorizon(roadmap_connection_point))
        {
          double dist_to_robot = (roadmap_connection_point - eigen_robot_position).norm();
          if (dist_to_robot < min_dist_to_robot)
          {
            min_dist_to_robot = dist_to_robot;
            global_path_robot_position.x = roadmap_connection_point.x();
            global_path_robot_position.y = roadmap_connection_point.y();
            global_path_robot_position.z = roadmap_connection_point.z();
          }
        }
      }
    }
  }

  // std::cout << "----------进入全局路径规划------------" << std::endl;

  //  其他机器人  会增加  很多新的信息  若是每次都重新规划导航点  可能会导致  导航 反复
  //  判定上一次探索状态     若为     远点导航   则 优先远点导航
  // 重写  沿用  前一次导航点逻辑
  last_robot_cell_ind_ =
      GetCellInd(global_path_robot_position.x, global_path_robot_position.y, global_path_robot_position.z);

  if ((robot_statu_ == RobotStatus::Exploring || robot_statu_ == RobotStatus::Global_tsp ||
       robot_statu_ == RobotStatus::Return_home) &&
      is_global_tsp == false)
  {
    // std::cout << "跳出全局路径规划    执行局部路径" << std::endl;
    robot_statu_ = RobotStatus::Exploring;
    Request_breakpoint_count_ = 0;  //  坏点计数
    far_breakpoint_coint_ = 0;
    return global_path;  //此时的全局 路径为空
  }

  if (robot_statu_ == RobotStatus::Far_planner)
  {
    // 上次为远点导航  本次判定逻辑：
    // 查询上次路径长度   及    网格点状态
    //  获取上次      的导航点    和   最终的目标点
    exploration_path_ns::ExplorationPath last_path = last_global_goal_path_;
    // std::cout << "===前一次为  far  判定   是否沿用===" << std::endl;
    if (last_path.GetLength() > 2)
    {
      // goal_position_  上一次的目标
      int last_goal_cell_id = GetCellInd(goal_position_.x, goal_position_.y, goal_position_.z);
      if (subspaces_world_->GetCell(last_goal_cell_id).GetStatus() ==
          CellStatus::EXPLORING)  //目标点的全局还是探索状态，则选用之前计算的结果
      {
        //查询 当前位置到  导航点距离  大于     1.5* kcellsize   就继续使用 前一个导航点
        // 使用  目标网格 点 在  merger  graph    上计算  当前 位置  和  上一次 的目标点  goal_position_   之间的路径
        // 传出即可
        // std::cout << "===上一次目标子网格状态为  探索===" << std::endl;
        geometry_msgs::msg::Point goal_position_graph = goal_position_;

        //  判定  目标网格的目标点     在 merger graph  上  是否能够到达
        bool is_reachable = merger_graph->IsPositionReachableIkdtree(goal_position_graph);
        //  判定距离  目标点 和当前  距离  
        double dist_robot_goal = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
            goal_position_, robot_position_);
        // 能够到达 且 距离 小 与  15 厘米 则 将网格状态置为 已探索  . 跳出 
        if(is_reachable && dist_robot_goal<0.15)
        {
          subspaces_world_->GetCell(last_goal_cell_id).SetStatus(CellStatus::COVERED);
           subspaces_->GetCell(last_goal_cell_id).SetStatus(CellStatus::COVERED);
            subspaces_local_->GetCell(last_goal_cell_id).SetStatus(CellStatus::COVERED);
          // 不执行导航
          is_reachable = false;
        }

        //  能够到达    计算路径导航    不能到达       切换状态
        if (is_reachable)
        {
          exploration_path_ns::Node node_robot(global_path_robot_position, exploration_path_ns::NodeType::ROBOT);
          node_robot.global_subspace_index_ =
              GetCellInd(global_path_robot_position.x, global_path_robot_position.y, global_path_robot_position.z);
          global_path.Append(node_robot);
          //目标点在局部范围内  就直接将目标点做为导航点
          exploration_path_ns::Node node_goal(goal_position_,
                                              exploration_path_ns::NodeType::GLOBAL_VIEWPOINT);  //目标点
          node_goal.global_subspace_index_ = GetCellInd(goal_position_.x, goal_position_.y, goal_position_.z);
          // 判定当前  机器人是否在
          //  计算  当前位置  和 机器人  之间距离
          nav_msgs::msg::Path merger_graph_path;  //路径
          merger_graph->GetShortestPath(global_path_robot_position, goal_position_, true, merger_graph_path,
                                        true);  //得到两个位置的   连接路径
          // std::cout << "当前位置到目标路径  far_path.size()  =  " << merger_graph_path.poses.size() << std::endl;
          int goal_cell_ind = GetCellInd(goal_position_.x, goal_position_.y, goal_position_.z);
          ordered_cell_indices.push_back(goal_cell_ind);
          // Fill in the path in between        填充中间的路径   不存在路径  就无所谓
          if (merger_graph_path.poses.size() >= 2)  //两点之间的路径
          {
            for (int j = 1; j < merger_graph_path.poses.size() - 1; j++)
            {
              geometry_msgs::msg::Point node_position;
              node_position = merger_graph_path.poses[j].pose.position;
              exploration_path_ns::Node merger_graph_node(node_position,
                                                          exploration_path_ns::NodeType::GLOBAL_VIA_POINT);
              merger_graph_node.keypose_graph_node_ind_ =
                  static_cast<int>(merger_graph_path.poses[j].pose.orientation.x);
              global_path.Append(merger_graph_node);
            }
          }
          // 添加 目标点
          global_path.Append(node_goal);
          // std::cout << "当前位置到目标路径global_path.size()  =  " << global_path.nodes_.size() << std::endl;
          //更新机器人状态  远点 导航   导航到  待 探索的子网格
          robot_statu_ = RobotStatus::Far_planner;
          last_global_goal_path_ = global_path;
          return global_path;
        }
      }
    }

    if (is_global_tsp == false)  // 存在局部规划
    {
      Request_breakpoint_count_ = 0;
      robot_statu_ = RobotStatus::Exploring;
      far_breakpoint_coint_ = 0;
      return global_path;
    }
    robot_statu_ = RobotStatus::Exploring;
  }

  std::vector<geometry_msgs::msg::Point> exploring_cell_positions;        //探索  单元_本地   位置数据
  std::vector<geometry_msgs::msg::Point> exploring_cell_positions_world;  //探索  单元_世界
  std::vector<int> exploring_cell_indices;  //探索单元  ID（即哪些单元是作为探索的目标   本地可到达探索点）
  std::vector<int> exploring_cell_indices_world;  //探索单元  ID（即哪些单元是作为探索的目标，本地全局可到达探索点）
  std::vector<int> exploring_cell_ID;                             //判定目标
  std::map<int, std::vector<int>> robotid_neighbor_cell_indices;  //机器人 编号  和  邻接网格
  //   融合  所有机器人的   邻接网格       即  所有 机器人的  邻接网格  都不能加入 Mdvrp  的计算
  std::vector<int> neighbor_cell_indices_all;

  // 其他机器人  位置  other_robot_position_map
  // 其他机器人  邻接网格编号  robotid_neighbor_cell_indices
  for (auto& pos : other_robot_position_map)
  {
    std::vector<int> temp_neighbor_cell_indices;
    int N = KNearbyGridNum / 2;  // 5/2
    int M = 1;
    GetNeighborCellIndices(pos.second, Eigen::Vector3i(N, N, M), temp_neighbor_cell_indices);
    robotid_neighbor_cell_indices[pos.first] = temp_neighbor_cell_indices;
    // 使用   temp_neighbor_cell_indices    与   neighbor_cell_indices_all  做差
    std::vector<int> temp_neighbor_cell_indices_add_;
    misc_utils_ns::SetDifference(temp_neighbor_cell_indices, neighbor_cell_indices_all,
                                 temp_neighbor_cell_indices_add_);
    // 结果插入  neighbor_cell_indices_all
    for (auto& ind : temp_neighbor_cell_indices_add_)
    {
      neighbor_cell_indices_all.push_back(ind);
    }
    // neighbor_cell_indices_all.insert(neighbor_cell_indices_all.end(), temp_neighbor_cell_indices_add.begin(),
    //                                  temp_neighbor_cell_indices_add.end());
  }
  // 本地机器人位置    global_path_robot_position
  // 本地机器人网格    neighbor_cell_indices_  插入
  // 排除其他机器人的邻接  网格
  // std::cout << "其他机器人个数" << neighbor_cell_indices_all.size() << std::endl;
  std::vector<int> temp_neighbor_cell_indices_add;
  std::vector<int> cur_neighbor_cell_indices(neighbor_cell_indices_);
  misc_utils_ns::SetDifference(neighbor_cell_indices_all, cur_neighbor_cell_indices, temp_neighbor_cell_indices_add);
  // 结果插入  neighbor_cell_indices_all
  neighbor_cell_indices_all = temp_neighbor_cell_indices_add;

  // 从其他机器人那里减去  自己相同的机器人
  // 用于共享  本地探索点
  // 用于 驱动  全局可到达目标点
  // 得到    exploring_cell_positions_world    exploring_cell_indices_world
  int count_exploring_cell = 0;
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)  //对于子空间的单元
  {
    if (subspaces_world_->GetCell(i).GetStatus() == CellStatus::EXPLORING)  //如果是全局的     是探索状态的  单元 且
                                                                            //网格点不为空 且能到达   放入探索单元集里面
    {
      //排除  在  多机器人  的   邻接网格内
      if (std::find(neighbor_cell_indices_all.begin(), neighbor_cell_indices_all.end(), i) ==
          neighbor_cell_indices_all.end())
      {
        count_exploring_cell++;
        if (!use_keypose_graph_ || keypose_graph == nullptr ||
            keypose_graph->GetNodeNum() == 0)  //不用关键位姿图 或 关键位姿图才  初始化
        {
          // Use straight line connection  使用直线连接
          exploring_cell_positions_world.push_back(GetCellPosition(i));  //将当前单位放入探索单元集
          exploring_cell_indices_world.push_back(i);
        }
        else
        {
          // 取网格    中心点  做目标点
          Eigen::Vector3d connection_point =
              subspaces_->GetCell(i).GetRoadmapConnectionPoint();  //子空间的路线图连接点 就是距离cell中心最近的视点的值
          geometry_msgs::msg::Point connection_point_geo;
          connection_point_geo.x = connection_point.x();
          connection_point_geo.y = connection_point.y();
          connection_point_geo.z = connection_point.z();

          bool reachable = false;  //初始定义当前子空间单元不可通行
          if (merger_graph->IsPositionReachableIkdtree(connection_point_geo))
          {
            reachable = true;
          }
          else  //当不可通行时寻找此单元其他关键位姿结点 与当前    connection_point_geo   最近的点
          {
            // 检查此单元格中的所有    merger graph  的  节点，以查看是否存在任何连接的节点
            double min_dist = DBL_MAX;
            double min_dist_node_ind = -1;
            for (const auto& node_ind : subspaces_->GetCell(i).GetMergerGraphNodeIndices())
            {
              geometry_msgs::msg::Point node_position = merger_graph->GetNodePosition(node_ind);  //单元内 存在于图上的结点集
              double dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
                  node_position, connection_point_geo);
              bool is_connected = merger_graph->GetNodeIsConnected(node_ind);  //是否是连接的
              if (is_connected && dist < min_dist)
              {
                min_dist = dist;
                min_dist_node_ind = node_ind;
              }
            }
            if (min_dist_node_ind >= 0 && min_dist_node_ind < merger_graph->GetNodeNum())
            {
              reachable = true;
              connection_point_geo = merger_graph->GetNodePosition(min_dist_node_ind);
            }
          }
          if (reachable)
          {
            exploring_cell_positions_world.push_back(connection_point_geo);  //得到探索单元的位置信息
            exploring_cell_indices_world.push_back(i);
          }
        }
      }
    }
  }

  num_world_neighbor_cell_reachable = exploring_cell_indices_world.size();

  //  计算 做 Mdvrp 的 机器人  个数
  //  若是   单机器人  则  使用  TSP  求解
  //  若是   多机器人  则  需要   机器人位置 在  exploring_cell_positions_world   上  记录映射
  std::vector<int> robot_position_id_on_exploring_cell_positions_world;  // 机器人位置  在
                                                                         // exploring_cell_positions_world上的 索引

  // 首先  添加 当前机器人位置
  std::vector<int> robot_ids_sorted;
  robot_ids_sorted.push_back(cur_robot_id_);
  for (auto& temp : other_robot_position_map)
  {
    robot_ids_sorted.push_back(temp.first);
  }
  std::sort(robot_ids_sorted.begin(), robot_ids_sorted.end());
  robot_ids_sorted.erase(std::unique(robot_ids_sorted.begin(), robot_ids_sorted.end()), robot_ids_sorted.end());

  for (int i = 0; i < robot_ids_sorted.size(); i++)
  {
    int robot_id = robot_ids_sorted[i];
    geometry_msgs::msg::Point robot_pos;
    if (robot_id == cur_robot_id_)
    {
      robot_pos = global_path_robot_position;
    }
    else
    {
      robot_pos = other_robot_position_map[robot_id];
    }
    robot_position_id_on_exploring_cell_positions_world.push_back(exploring_cell_indices_world.size());
    exploring_cell_positions_world.push_back(robot_pos);
    int temp_robot_index = GetCellInd(robot_pos.x, robot_pos.y, robot_pos.z);
    exploring_cell_indices_world.push_back(temp_robot_index);
  }

  std::vector<std::vector<int>> distance_matrix;
  std::vector<std::vector<int>> distance_matrix_world(exploring_cell_positions_world.size(),
                                                      std::vector<int>(exploring_cell_positions_world.size(), 0));

  for (int i = 0; i < exploring_cell_positions_world.size(); i++)
  {
    for (int j = 0; j < i; j++)
    {
      if (!use_keypose_graph_ || keypose_graph == nullptr ||
          keypose_graph->GetNodeNum() == 0)  //不使用关键位姿图   直接用两点 直线距离构造边
      {
        // Use straight line connection     初始化  使用直线连接
        distance_matrix_world[i][j] =
            static_cast<int>(10 * misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
                                      exploring_cell_positions_world[i], exploring_cell_positions_world[j]));
      }
      else
      {
        // 骨架图加速：优先用全源最短路径缓存
        bool used_skeleton = false;
        if (skeleton_graph && skeleton_graph_ns::SkeletonGraph::enabled_ &&
            skeleton_graph->IsCellInGraph(exploring_cell_indices_world[i]) &&
            skeleton_graph->IsCellInGraph(exploring_cell_indices_world[j]))
        {
          double dist = skeleton_graph->GetShortestPathDist(
              exploring_cell_indices_world[i], exploring_cell_indices_world[j]);
          if (dist > 0)  // valid path found (>0), skip DBL_MAX / -1
          {
            distance_matrix_world[i][j] = static_cast<int>(10 * dist);
            used_skeleton = true;
          }
        }
        if (!used_skeleton)
        {
          // 回退到 merger graph 路径搜索
          nav_msgs::msg::Path path_tmp;
          distance_matrix_world[i][j] = static_cast<int>(
              10 * merger_graph->GetShortestPath(exploring_cell_positions_world[i], exploring_cell_positions_world[j],
                                                 false, path_tmp, true));
        }
      }
    }
  }
  //对角化
  for (int i = 0; i < exploring_cell_positions_world.size(); i++)
  {
    for (int j = i + 1; j < exploring_cell_positions_world.size(); j++)
    {
      distance_matrix_world[i][j] = distance_matrix_world[j][i];
    }
  }

  if (robot_position_id_on_exploring_cell_positions_world.size() > 1)
  {
    is_fuse_ = true;
  }

  //  返回起点
  return_home_ = false;

  if (is_global_tsp == true && num_world_neighbor_cell_reachable == 0)
  {
    geometry_msgs::msg::Point home_position;  //返回路径

    nav_msgs::msg::Path return_home_path;
    if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
    {
      geometry_msgs::msg::PoseStamped robot_pose;
      robot_pose.pose.position = robot_position_;

      geometry_msgs::msg::PoseStamped home_pose;
      home_pose.pose.position = home_position;
      return_home_path.poses.push_back(robot_pose);
      return_home_path.poses.push_back(home_pose);
    }
    else
    {
      // 寻找距离起点最近的  连通的 节点
      geometry_msgs::msg::Point start_position = merger_graph->GetFirstKeyposePosition();
      int start_ind = merger_graph->GetClosestNodeIndIkdtree(start_position);
      home_position = merger_graph->GetNodePosition(start_ind);
      merger_graph->GetShortestPath(global_path_robot_position, home_position, true, return_home_path, false);
      if (return_home_path.poses.size() >= 2)
      {
        global_path.FromPath(return_home_path);
        global_path.nodes_.front().type_ = exploration_path_ns::NodeType::ROBOT;

        for (int i = 1; i < global_path.nodes_.size() - 1; i++)
        {
          global_path.nodes_[i].type_ = exploration_path_ns::NodeType::GLOBAL_VIA_POINT;
        }
        global_path.nodes_.back().type_ = exploration_path_ns::NodeType::HOME;
        // Make it a loop
        for (int i = global_path.nodes_.size() - 2; i >= 0; i--)
        {
          global_path.Append(global_path.nodes_[i]);
        }
      }
      else
      {
        // ROS_ERROR("Cannot find path home");
        // TODO: find a path
      }
    }
    //更新机器人状态
    robot_statu_ = RobotStatus::Return_home;
    far_breakpoint_coint_ = 0;
    return global_path;  //返回一个路径
  }

  return_home_ = false;

  std::vector<int> node_index;
  if (!is_fuse_)  //融合失败  做      TSP    用全局可到达      exploring_cell_positions_world
  {
    /****** Solve the TSP  求解   旅行商(TSP)问题 优化 ******/
    tsp_solver_ns::DataModel data_model;                           //数据模型
    data_model.distance_matrix = distance_matrix_world;            //距离矩阵
    data_model.depot = exploring_cell_positions_world.size() - 1;  //起点  就是  最后的一个
    tsp_solver_ns::TSPSolver tsp_solver(data_model);
    tsp_solver.Solve();
    tsp_solver.getSolutionNodeIndex(node_index, false);
    exploring_cell_indices = exploring_cell_indices_world;
    exploring_cell_positions = exploring_cell_positions_world;
    distance_matrix = distance_matrix_world;
    ordered_cell_indices.clear();
  }
  else
  {  //成功做 Mdvrp
    int robot_num = robot_position_id_on_exploring_cell_positions_world.size();
    tsp_solver_ns::DataModel data_model;                 //数据模型
    data_model.distance_matrix = distance_matrix_world;  //距离矩阵
    data_model.num_vehicles = robot_num;
    for (int i = 0; i < robot_num; i++)
    {
      int id = robot_position_id_on_exploring_cell_positions_world[i];
      std::pair<int, int> id_(id, id);  //起点终点下标  ，每个车的起点和终点都是一样的   所以相同
      data_model.depot_M.emplace_back(id_);              //  第一个是本地机器人位置
      data_model.depot_M_other_strategy.push_back(id_);  //  第一个是本地机器人位置
    }
    data_model.is_MTSP = true;  //默认是  false    所以在  TSP  上不用修改。
    data_model.Allocation_strategy = allocation_strategy_;
    data_model.robot_ids = robot_ids_sorted;
    data_model.current_robot_id = cur_robot_id_;
    tsp_solver_ns::TSPSolver tsp_solver(data_model);
    tsp_solver.Solve();
    tsp_solver.getSolutionNodeIndex(node_index, false);  //
    ordered_cell_indices.clear();
    //更新 变量方便后续处理
    exploring_cell_indices = exploring_cell_indices_world;
    exploring_cell_positions = exploring_cell_positions_world;
    distance_matrix = distance_matrix_world;
  }

  // Add the first node in the end to make it a loop  最后添加第一个节点，使其成为循环        TSP结果
  if (!node_index.empty())  //从  TSP 求解出来的  node_index  由起点->终点前一个       起点  终点是一个点
                            //终点需要自己添加变成环
  {
    node_index.push_back(node_index[0]);  //添加起点 成为循环
  }
  //打印  机器人序号
  int robot_cell_id;
  if (!is_fuse_)  //融合失败  做    TSP   不打印  TSP 结果
  {
    robot_cell_id = exploring_cell_indices.back();
  }
  else
  {
    //做  Mdvrp  打印了  robot_position_id_on_exploring_cell_positions_world.front()  第一个是本地机器人
    robot_cell_id = exploring_cell_indices_world[robot_position_id_on_exploring_cell_positions_world.front()];
  }

  // Mdvrp    走这个路线
  if (is_fuse_)  //  融合成功走这个路线     选一个做目标点  传出
  {
    if (!use_keypose_graph_ || keypose_graph == nullptr ||
        merger_graph->GetNodeNum() == 0)  //如果  不使用关键位姿图或者是初始化的时候
    {
      for (int i = 0; i < node_index.size(); i++)  //对于每个排序的点
      {
        int cell_ind = node_index[i];
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position = exploring_cell_positions[cell_ind];
        exploration_path_ns::Node node(exploring_cell_positions[cell_ind],
                                       exploration_path_ns::NodeType::GLOBAL_VIEWPOINT);
        node.global_subspace_index_ = exploring_cell_indices[cell_ind];
        global_path.Append(node);
        ordered_cell_indices.push_back(exploring_cell_indices[cell_ind]);
      }
    }
    else  //如果使用关键位姿图
    {
      //  探索补偿
      int robot_num = robot_position_id_on_exploring_cell_positions_world.size();  //机器人数量
      int exploring_num =
          distance_matrix_world.size() - robot_position_id_on_exploring_cell_positions_world.size();  //外部未探索点数量
      exploration_path_ns::Node node_goal;
      int goal_ind;
      if (node_index.size() <= 2 && exploring_num > 0)  // mdvrp       没有给本地机器人分配目标点
                                                        // 但还是存在可达到的未探索位置     修正  探索点
                                                        // 给入（贪婪，最近的一个未探索点当作目标点）
      {
        std::vector<int> distance_vector = distance_matrix_world[robot_position_id_on_exploring_cell_positions_world.front()];
        std::vector<geometry_msgs::msg::Point> goal_position;
        std::vector<int> goal_position_distance;
        std::vector<int> goal_MTSP_index;
        for (int i = 0; i < distance_vector.size(); i++)
        {
          bool flag = false;
          for (int j = 0; j < robot_num; j++)
          {
            if (i == robot_position_id_on_exploring_cell_positions_world[j])  //排除  机器人位置
            {
              flag = true;
            }
          }
          if (!flag)
          {
            goal_position.push_back(exploring_cell_positions_world[i]);
            goal_position_distance.push_back(distance_vector[i]);
            goal_MTSP_index.push_back(i);
          }
        }

        int minPosition = std::min_element(goal_position_distance.begin(), goal_position_distance.end()) - goal_position_distance.begin();  //得到最小点的索引

        exploration_path_ns::Node node_goal_(Eigen::Vector3d(goal_position[minPosition].x, goal_position[minPosition].y, goal_position[minPosition].z));  //当前位置
        node_goal_.type_ = exploration_path_ns::NodeType::GLOBAL_VIEWPOINT;
        node_goal_.global_subspace_index_ = GetCellInd(goal_position[minPosition].x, goal_position[minPosition].y, goal_position[minPosition].z);
        node_goal = node_goal_;
        goal_ind = goal_MTSP_index[minPosition];  // goal_ind  在
      }
      else if (node_index.size() <= 2 && exploring_num == 0)  // mtsp  没有目标点  则  return home ?还是跟踪其中一个机器人
      {
        geometry_msgs::msg::Point home_position;  //返回路径
        nav_msgs::msg::Path return_home_path;
        if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
        {
          geometry_msgs::msg::PoseStamped robot_pose;
          robot_pose.pose.position = robot_position_;

          geometry_msgs::msg::PoseStamped home_pose;
          home_pose.pose.position = home_position;
          return_home_path.poses.push_back(robot_pose);
          return_home_path.poses.push_back(home_pose);
        }
        else
        {
          home_position = merger_graph->GetFirstKeyposePosition();
          merger_graph->GetShortestPath(global_path_robot_position, home_position, true, return_home_path, false);
          if (return_home_path.poses.size() >= 2)
          {
            global_path.FromPath(return_home_path);
            global_path.nodes_.front().type_ = exploration_path_ns::NodeType::ROBOT;

            for (int i = 1; i < global_path.nodes_.size() - 1; i++)
            {
              global_path.nodes_[i].type_ = exploration_path_ns::NodeType::GLOBAL_VIA_POINT;
            }
            global_path.nodes_.back().type_ = exploration_path_ns::NodeType::HOME;
            // Make it a loop
            for (int i = global_path.nodes_.size() - 2; i >= 0; i--)
            {
              global_path.Append(global_path.nodes_[i]);
            }
          }
          else
          {
            // ROS_ERROR("Cannot find path home");
            // TODO: find a path
          }
        }
        is_fuse_ = false;
        robot_statu_ = RobotStatus::Return_home;
        // return_home_=true;
        return global_path;  //返回一个路径
      }
      else
      {
        // //加入目标
        int goal_ind_ = node_index[1];  //需要比较前后两个点谁最小么？
        //比较  循环结果  前后两个点谁的距离较近
        int goal_ind_compare = node_index[node_index.size() - 2];
        if (distance_matrix_world[robot_position_id_on_exploring_cell_positions_world[0]][goal_ind_] >
            distance_matrix_world[robot_position_id_on_exploring_cell_positions_world[0]][goal_ind_compare])
        {
          goal_ind_ = goal_ind_compare;
        }

        exploration_path_ns::Node node_goal_(Eigen::Vector3d(exploring_cell_positions[goal_ind_].x,
                                                             exploring_cell_positions[goal_ind_].y,
                                                             exploring_cell_positions[goal_ind_].z));  //当前位置
        node_goal_.type_ = exploration_path_ns::NodeType::GLOBAL_VIEWPOINT;
        node_goal_.global_subspace_index_ =
            GetCellInd(exploring_cell_positions[goal_ind_].x, exploring_cell_positions[goal_ind_].y,
                       exploring_cell_positions[goal_ind_].z);
        node_goal = node_goal_;
        goal_ind = goal_ind_;
      }

      int robot_ind = node_index[0];
      geometry_msgs::msg::Point cur_position;
      cur_position = exploring_cell_positions[robot_ind];
      exploration_path_ns::Node node_local(Eigen::Vector3d(exploring_cell_positions[robot_ind].x,
                                                           exploring_cell_positions[robot_ind].y,
                                                           exploring_cell_positions[robot_ind].z));  //当前位置
      node_local.type_ = exploration_path_ns::NodeType::ROBOT;
      node_local.global_subspace_index_ = GetCellInd(exploring_cell_positions[robot_ind].x, exploring_cell_positions[robot_ind].y, exploring_cell_positions[robot_ind].z);
      global_path.Append(node_local);

      nav_msgs::msg::Path merger_graph_path;  //路径
      merger_graph->GetShortestPath(cur_position, exploring_cell_positions[goal_ind], true, merger_graph_path,
                                    true);  //得到两个位置的   连接路径
      ordered_cell_indices.push_back(exploring_cell_indices[robot_ind]);
      ordered_cell_indices.push_back(exploring_cell_indices[goal_ind]);

      if (merger_graph_path.poses.size() >= 2)  //两点之间的路径
      {
        for (int j = 1; j < merger_graph_path.poses.size() - 1; j++)
        {
          geometry_msgs::msg::Point node_position;
          node_position = merger_graph_path.poses[j].pose.position;
          exploration_path_ns::Node merger_graph_node(node_position, exploration_path_ns::NodeType::GLOBAL_VIA_POINT);
          merger_graph_node.keypose_graph_node_ind_ = static_cast<int>(merger_graph_path.poses[j].pose.orientation.x);
          global_path.Append(merger_graph_node);
        }
      }
      // 添加 目标点
      global_path.Append(node_goal);
      //更新机器人状态  远点 导航   导航到  待 探索的子网格
      robot_statu_ = RobotStatus::Far_planner;
      last_global_goal_path_ = global_path;
      goal_position_ = exploring_cell_positions[goal_ind];
      return global_path;
    }
  }

  // 单车  使用   TSP  走这个路线
  if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)  //如果  不使用关键位姿图
  {
    for (int i = 0; i < node_index.size(); i++)  //对于每个排序的点
    {
      int cell_ind = node_index[i];
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position = exploring_cell_positions[cell_ind];
      exploration_path_ns::Node node(exploring_cell_positions[cell_ind],
                                     exploration_path_ns::NodeType::GLOBAL_VIEWPOINT);
      node.global_subspace_index_ = exploring_cell_indices[cell_ind];
      global_path.Append(node);
      ordered_cell_indices.push_back(exploring_cell_indices[cell_ind]);
    }
  }
  else  //如果使用关键位姿图
  {
    geometry_msgs::msg::Point cur_position;
    geometry_msgs::msg::Point next_position;
    int cur_keypose_id;
    int next_keypose_id;
    int cur_ind;
    int next_ind;

    for (int i = 0; i < node_index.size() - 1; i++)
    {
      cur_ind = node_index[i];       //当前位置
      next_ind = node_index[i + 1];  //下一个位置
      cur_position = exploring_cell_positions[cur_ind];
      next_position = exploring_cell_positions[next_ind];

      nav_msgs::msg::Path keypose_path;  //路径
      // 阶段 3.3c: 优先用 skeleton_graph 加速路径填充
      bool used_skeleton_path = false;
      if (skeleton_graph && skeleton_graph_ns::SkeletonGraph::enabled_ &&
          !skeleton_graph->IsDegraded() && skeleton_graph->GetNodeNum() > 0 &&
          skeleton_graph->IsCellInGraph(exploring_cell_indices[cur_ind]) &&
          skeleton_graph->IsCellInGraph(exploring_cell_indices[next_ind]))
      {
        used_skeleton_path = skeleton_graph->GetShortestPath(
            cur_position, next_position, true, keypose_path, *merger_graph);
      }
      if (!used_skeleton_path)
      {
        // 回退到 keypose_graph 路径搜索
        keypose_graph->GetShortestPath(cur_position, next_position, true, keypose_path, true);
      }
      exploration_path_ns::Node node(Eigen::Vector3d(cur_position.x, cur_position.y, cur_position.z));  //当前位置
      if (i == 0)
      {
        node.type_ = exploration_path_ns::NodeType::ROBOT;  //机器人位置
      }
      else
      {
        node.type_ = exploration_path_ns::NodeType::GLOBAL_VIEWPOINT;  //全局视点的位置
      }
      node.global_subspace_index_ = exploring_cell_indices[cur_ind];
      global_path.Append(node);

      ordered_cell_indices.push_back(exploring_cell_indices[cur_ind]);

      // Fill in the path in between        填充中间的路径   不存在路径  就无所谓
      if (keypose_path.poses.size() >= 2)  //两点之间的路径
      {
        for (int j = 1; j < keypose_path.poses.size() - 1; j++)
        {
          geometry_msgs::msg::Point node_position;
          node_position = keypose_path.poses[j].pose.position;
          exploration_path_ns::Node keypose_node(node_position, exploration_path_ns::NodeType::GLOBAL_VIA_POINT);
          keypose_node.keypose_graph_node_ind_ = static_cast<int>(keypose_path.poses[j].pose.orientation.x);
          global_path.Append(keypose_node);
        }
      }
    }
    // Append the robot node to the end
    if (!global_path.nodes_.empty())
    {
      global_path.Append(global_path.nodes_[0]);
    }
  }
  //更新机器人状态
  robot_statu_ = RobotStatus::Global_tsp;
  last_global_goal_path_ = global_path;
  return global_path;
}
}  // namespace grid_world_ns