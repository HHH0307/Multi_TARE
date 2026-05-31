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
  keypose_graph_node_indices_.clear();
  MTSP_graph_index_find_exploring_cell_ = -1; // 新增
}

bool Cell::IsCellConnected(int cell_ind)
{
  if (std::find(connected_cell_indices_.begin(), connected_cell_indices_.end(), cell_ind) !=
      connected_cell_indices_.end())
  {
    return true;
  }
  else
  {
    return false;
  }
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

void GridWorld::UpdateCellStatus(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
{
  int exploring_count = 0;
  int unseen_count = 0;
  int covered_count = 0;
  for (int i = 0; i < subspaces_->GetCellNumber(); ++i)
  {
    if (subspaces_local_->GetCell(i).GetStatus() == CellStatus::EXPLORING)
    {
      exploring_count++;
    }
    else if (subspaces_local_->GetCell(i).GetStatus() == CellStatus::UNSEEN)
    {
      unseen_count++;
    }
    else if (subspaces_local_->GetCell(i).GetStatus() == CellStatus::COVERED)
    {
      covered_count++;
    }
  }

  for (const auto& cell_ind : neighbor_cell_indices_)
  {
    subspaces_->GetCell(cell_ind).ClearViewPointIndices();
  }
  for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
  {
    geometry_msgs::msg::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);
    Eigen::Vector3i sub = subspaces_->Pos2Sub(Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z));
    if (subspaces_->InRange(sub))
    {
      int cell_ind = subspaces_->Sub2Ind(sub);
      AddViewPointToCell(cell_ind, viewpoint_ind);
      viewpoint_manager->SetViewPointCellInd(viewpoint_ind, cell_ind);
    }
    else
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("standalone_logger"), "subspace sub out of bound: " << sub.transpose());
    }
  }

  for (const auto& cell_ind : neighbor_cell_indices_)
  {
    if (subspaces_local_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED_BY_OTHERS)
    {
      continue;
    }
    int candidate_count = 0;
    int selected_viewpoint_count = 0;
    int above_big_threshold_count = 0;
    int above_small_threshold_count = 0;
    int above_frontier_threshold_count = 0;
    int highest_score_viewpoint_ind = -1;
    int highest_score = -1;
    for (const auto& viewpoint_ind : subspaces_->GetCell(cell_ind).GetViewPointIndices())
    {
      MY_ASSERT(viewpoint_manager->IsViewPointCandidate(viewpoint_ind));
      candidate_count++;
      if (viewpoint_manager->ViewPointSelected(viewpoint_ind))
      {
        selected_viewpoint_count++;
      }
      if (viewpoint_manager->ViewPointVisited(viewpoint_ind))
      {
        continue;
      }
      int score = viewpoint_manager->GetViewPointCoveredPointNum(viewpoint_ind);
      int frontier_score = viewpoint_manager->GetViewPointCoveredFrontierPointNum(viewpoint_ind);
      if (score > highest_score)
      {
        highest_score = score;
        highest_score_viewpoint_ind = viewpoint_ind;
      }
      if (score > kMinAddPointNumSmall)
      {
        above_small_threshold_count++;
      }
      if (score > kMinAddPointNumBig)
      {
        above_big_threshold_count++;
      }
      if (frontier_score > kMinAddFrontierPointNum)
      {
        above_frontier_threshold_count++;
      }
    }
    // Exploring to Covered 新修改
    if (subspaces_local_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING &&
        above_frontier_threshold_count < kCellExploringToCoveredThr &&
        above_small_threshold_count < kCellExploringToCoveredThr && selected_viewpoint_count == 0 &&
        candidate_count > 0)
    {
      subspaces_local_->GetCell(cell_ind).SetStatus(CellStatus::COVERED); // 新增
      if(subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN || subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING) // 新增 判断
      {
        subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
      }
    }
    // Covered to Exploring 新修改
    else if (subspaces_local_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED &&
             (above_big_threshold_count >= kCellCoveredToExploringThr ||
              above_frontier_threshold_count >= kCellCoveredToExploringThr))
    {
      subspaces_local_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING); // 新增
      if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN) // 新增
      {
        subspaces_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING);
      }
      almost_covered_cell_indices_.push_back(cell_ind);
    }
    // Exploring to Almost covered 新修改
    else if (subspaces_local_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING && selected_viewpoint_count == 0 &&
             candidate_count > 0)
    {
      almost_covered_cell_indices_.push_back(cell_ind);
    }
    // 新修改
    else if (subspaces_local_->GetCell(cell_ind).GetStatus() != CellStatus::COVERED && selected_viewpoint_count > 0)
    {
      subspaces_local_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING);
      if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN) // 新增
      {
        subspaces_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING); // 新增
      }
      almost_covered_cell_indices_.erase(
          std::remove(almost_covered_cell_indices_.begin(), almost_covered_cell_indices_.end(), cell_ind),
          almost_covered_cell_indices_.end());
    }
    // 新修改
    else if (subspaces_local_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING && candidate_count == 0)
    {
      // First visit
      if (subspaces_->GetCell(cell_ind).GetVisitCount() == 1 &&
          subspaces_->GetCell(cell_ind).GetGraphNodeIndices().empty())
      {
        subspaces_local_->GetCell(cell_ind).SetStatus(CellStatus::COVERED); // 新增 
        if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN || subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING)  // 新增
        {
          subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
        }
      }
      else
      {
        geometry_msgs::msg::Point cell_position = subspaces_->GetCell(cell_ind).GetPosition();
        double xy_dist_to_robot = misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(cell_position, robot_position_);
        double z_dist_to_robot = std::abs(cell_position.z - robot_position_.z);
        if (xy_dist_to_robot < kCellSize && z_dist_to_robot < kCellHeight * 0.8)
        {
          subspaces_local_->GetCell(cell_ind).SetStatus(CellStatus::COVERED); // 新增
          if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN || subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING) // 新增循环
          {
            subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
          }
        }
      }
    }

    // 新修改
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
      subspaces_local_->GetCell(cell_ind).SetStatus(CellStatus::COVERED); // 新增
      if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN || subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING) // 新增
      {
        subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED); 
      }
      almost_covered_cell_indices_.erase(
          std::remove(almost_covered_cell_indices_.begin(), almost_covered_cell_indices_.end(), cell_ind),
          almost_covered_cell_indices_.end());
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
          std::find(from_cell_connected_cell_indices.begin(), from_cell_connected_cell_indices.end(), to_cell_ind) !=
          from_cell_connected_cell_indices.end();
      bool backward_connected = std::find(to_cell_connected_cell_indices.begin(), to_cell_connected_cell_indices.end(),
                                          from_cell_ind) != to_cell_connected_cell_indices.end();

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
  if (debug)
  {
    std::cout << "[GW][Share] UpdateGridWorldCellFromOtherRobots: total=" << Update_Grid_World_ID_and_Statu_.size()
              << " unseen=" << count_unseen << " exploring=" << count_exploring << " covered=" << count_covered
              << " covered_by_others=" << count_covered_by_others << " nogo=" << count_nogo << std::endl;
  }
  Update_Grid_World_ID_and_Statu_.clear();  //更新一次就清空  重新接受
}

//添加函数   编码  MTSP  子图
std::vector<double> GridWorld::GetMtspSubgrapherCode(std::vector<geometry_msgs::msg::Point>& exploring_cell_positions,
                                                     std::vector<std::vector<int>>& distance_matrix)
{
  std::vector<double> subgrapher_code;
  subgrapher_code.push_back(exploring_cell_positions.size());
  //得到  3*n  个数据
  for (int i = 0; i < exploring_cell_positions.size(); i++)
  {
    subgrapher_code.push_back(exploring_cell_positions[i].x);
    subgrapher_code.push_back(exploring_cell_positions[i].y);
    subgrapher_code.push_back(exploring_cell_positions[i].z);
  }
  MY_ASSERT(exploring_cell_positions.size() == distance_matrix.size());  //断言  传入的矩阵    和位置的关系相同
  for (int i = 0; i < exploring_cell_positions.size(); i++)
  {
    for (int j = 0; j < exploring_cell_positions.size(); j++)
    {
      subgrapher_code.push_back(distance_matrix[i][j]);
    }
  }
  return subgrapher_code;  //返回个数       1+3*n+n*n
}

//添加函数    融合子图      融合成功  做MTSP                   融合失败   做TSP     子图都是完全图              一代
bool GridWorld::GetGlobalMtspGrapher_indices_positions_dismatrix(
    std::vector<int>& exploring_cell_indices,                          //本地探索单元  ID
    std::vector<geometry_msgs::msg::Point>& exploring_cell_positions,       //本地探索 点
    std::vector<std::vector<int>>& distance_matrix,                    //本地距离矩阵
    std::vector<int>& exploring_cell_indices_MTSP,                     //  做MTSP  的探索单元  ID
    std::vector<geometry_msgs::msg::Point>& exploring_cell_positions_MTSP,  //  做MTSP探索单元  ID
    std::vector<std::vector<int>>& distance_matrix_MTSP,               //做MTSP   距离矩阵
    std::vector<int>& robot_position_id_on_exploring_cell_positions_MTSP,
    std::map<int, std::vector<double>>& MTSP_subgrapher_map_,  //外部子图矩阵
    const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  /*                   融合  MTSP  子图
                         融图逻辑  ：
                         以本地子图为基础
                         对其他外来子图结点 进行判定   分为   1.本地已探索     2.需外部探索点
                         整体融合
                         条件：必须满足       子图中至少有一个外来结点在关键位姿图中   中才能融合
                         1.搜索到本地已探索的点时，  寻找本地探索点       替换边关系  ，选中逻辑
                         2.保留外部探索点的各自关系，组成探索图
  */

  //解码     MTSP_subgrapher_map_
  std::map<int, std::vector<geometry_msgs::msg::Point>> robot_ID_positions_map;
  std::map<int, std::vector<std::vector<int>>> robot_ID_dis_matrix_map;
  std::map<int, std::vector<geometry_msgs::msg::Point>> robot_ID_positions_on_keyposegrapher;  
  std::map<int, std::vector<int>>
      robot_ID_positions_ID_on_keyposegrapher; 
  std::map<int, std::vector<geometry_msgs::msg::Point>> robot_ID_positions_out_keyposegrapher;
  std::map<int, std::vector<int>>
      robot_ID_positions_ID_out_keyposegrapher;  
  std::map<int, std::vector<std::vector<int>>> Cell_ID_positions_global;  //用于做  MTSP的  单元  ID 和位置。
  std::map<int, bool> robot_ID_is_fuse;                                   //<机器人ID ， 是否融合成功 >
  fuse_grapher Fuse_Grapher_;                                             //融合图结构体
  int other_submtspgrapher_num;
  other_submtspgrapher_num = MTSP_subgrapher_map_.size();
  if (other_submtspgrapher_num > 0)  //当有外部输入  子图
  {
    for (auto itr : MTSP_subgrapher_map_)
    {
      robot_ID_is_fuse.insert(std::pair<int, bool>(itr.first, false));  //对每个外部子图都创建一个判定  初始化为   false
      int robot_id;
      robot_id = itr.first;
      int node_num = itr.second[0];  // 0  位是结点数量
      int count = 1;
      std::vector<geometry_msgs::msg::Point> positions;
      std::vector<std::vector<int>> dis_matrix(node_num, std::vector<int>(node_num, 0));

      geometry_msgs::msg::Point point_;
      for (int i = 1; i < itr.second.size(); i++)  //  3*n    是机器人位置信息   n*n  是距离矩阵
      {
        if (i <= 3 * node_num)  // 3*n
        {
          if (i % 3 == 1)
          {
            point_.x = itr.second[i];
          }
          else if (i % 3 == 2)
          {
            point_.y = itr.second[i];
          }
          else if (i % 3 == 0)
          {
            point_.z = itr.second[i];
            positions.push_back(point_);
          }
        }
        else
        {  //  构造   距离矩阵      n*n
          int node_i = (i - 3 * node_num - 1) / node_num;
          int node_j = (i - 3 * node_num - 1) % node_num;
          dis_matrix[node_i][node_j] = (int)itr.second[i];
        }
      }
      robot_ID_positions_map.insert(std::pair<int, std::vector<geometry_msgs::msg::Point>>(itr.first, positions));
      robot_ID_dis_matrix_map.insert(std::pair<int, std::vector<std::vector<int>>>(itr.first, dis_matrix));
    }

    //打印   本地子图个数 及结点  编号

    //打印接受子图个数   及结点编号

    //使用  robot_ID_positions_map   和    robot_ID_dis_matrix_map    进行融合
    //  使用     robot_ID_positions_map   分类   目标点     在关键位姿图上（已探索目标点）
    //  不在关键位姿图上（未探索区域）
    //得到  分类位置  索引  robot_ID_positions_ID_on_keyposegrapher         robot_ID_positions_ID_out_keyposegrapher
    // std::cout << std::endl << "\033[1;32m" << "分类目标点" << "\033[0m" << std::endl;
    for (auto itr : robot_ID_positions_map)  //   分类 目标点
    {
      //  根据位置反算单元   计算  单元里面是否存在   关键位姿图结点，  存在就是可以通行的
      //  对每一个外部       探索点
      std::vector<geometry_msgs::msg::Point> positions_on_keyposegrapher;   //在关键位姿图上
      std::vector<int> positions_ID_on_keyposegrapher;                 //位置索引
      std::vector<geometry_msgs::msg::Point> positions_out_keyposegrapher;  //不在关键位姿图上
      std::vector<int> positions_ID_out_keyposegrapher;                //位置索引

      //  对每一个外部       探索点
      for (int i = 0; i < itr.second.size(); i++)
      {
        int Cell_ID = GetCellInd(itr.second[i].x, itr.second[i].y, itr.second[i].z); 
        bool reachable = false;  //初始定义当前子空间单元不可通行
        if (keypose_graph->IsPositionReachable(itr.second[i]))
        {
          reachable = true;
          positions_on_keyposegrapher.push_back(itr.second[i]);  //加入
          positions_ID_on_keyposegrapher.push_back(i); 
        }
        else  
        {
          double min_dist = DBL_MAX;
          double min_dist_node_ind = -1;
          for (const auto& node_ind : subspaces_->GetCell(Cell_ID).GetGraphNodeIndices())  //相同  单元      存在图结点
          {
            geometry_msgs::msg::Point node_position = keypose_graph->GetNodePosition(node_ind);  //单元内  存在于图上的结点集
            double dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(node_position, itr.second[i]);
            if (dist < min_dist)  //找到当前单元图结点上      距离  外部给入探索点  最近的距离
            {
              min_dist = dist;
              min_dist_node_ind = node_ind;
            }
          }
          if (min_dist_node_ind >= 0 && min_dist_node_ind < keypose_graph->GetNodeNum())
          {
            reachable = true;
            positions_on_keyposegrapher.push_back(itr.second[i]);  //加入
            positions_ID_on_keyposegrapher.push_back(i);
          }
        }

        if (!reachable)  //不可到达，就是不在已探索地方，在外部
        {
          positions_out_keyposegrapher.push_back(itr.second[i]);
          positions_ID_out_keyposegrapher.push_back(i);
        }
      }
      if (positions_on_keyposegrapher.size() > 0)
      {
        robot_ID_positions_on_keyposegrapher.insert(
            std::pair<int, std::vector<geometry_msgs::msg::Point>>(itr.first, positions_on_keyposegrapher));
        robot_ID_positions_ID_on_keyposegrapher.insert(
            std::pair<int, std::vector<int>>(itr.first, positions_ID_on_keyposegrapher));
      }
      if (positions_out_keyposegrapher.size() > 0)
      {
        robot_ID_positions_out_keyposegrapher.insert(
            std::pair<int, std::vector<geometry_msgs::msg::Point>>(itr.first, positions_out_keyposegrapher));
        robot_ID_positions_ID_out_keyposegrapher.insert(
            std::pair<int, std::vector<int>>(itr.first, positions_ID_out_keyposegrapher));
      }
    }

    //打印
    for (auto itr : robot_ID_positions_map)
    {
      std::cout << "\033[1;32m"
                << "机器:" << itr.first << " 外部已探索数量："
                << robot_ID_positions_ID_on_keyposegrapher[itr.first].size() << "\033[0m" << std::endl;
      std::cout << "\033[1;32m"
                << "机器:" << itr.first << " 外部未探索数量："
                << robot_ID_positions_ID_out_keyposegrapher[itr.first].size() << "\033[0m" << std::endl;
      std::cout << "\033[1;32m"
                << "结点CEll_id  :   "
                << "\033[0m" << std::endl;
      for (int i = 0; i < itr.second.size(); i++)
      {
        int Cell_ID = GetCellInd(itr.second[i].x, itr.second[i].y, itr.second[i].z);  //得到 外部探索点对应的 单元ID
        std::cout << "\033[1;32m" << Cell_ID << "\033[0m" << std::endl;
      }
    }
    //  根据分类位姿索引   以本地地图为基准   加入 子图的结点（外部探索点）        持续更新探索单元ID  探索位置数据
    //  及其对应的距离矩阵
    // 对每个已探索点的边关系          替换          本地探索点，并把找到的本地探索点             加入到全局探索结点
    //检测  外部传入已经探索   添加结点和边  到图中
    std::cout << std::endl
              << "\033[1;32m"
              << " 外部已探索点 融合"
              << "\033[0m" << std::endl;
    for (auto itr : robot_ID_positions_ID_on_keyposegrapher)
    {
      if (itr.second.empty())  //若是  外部已探索点集   没有 ,  说明  两个机器人没有共同区域  不融合
      {
        continue;
        // std::cout << std::endl << "\033[1;32m"<<"不与机器人:" << itr.first<< " 融合"
        // <<robot_ID_positions_ID_on_keyposegrapher[itr.first].size()<< "\033[0m" << std::endl;
      }
      for (int i = 0; i < itr.second.size(); i++)
      {
        // itr.second[i]    代表  的  robot_ID_positions_on_keyposegrapher   的索引
        // robot_ID_positions_on_keyposegrapher.second[itr.second[i]]   得到位置
        //  寻求           外部传入已探索点的边         替代
        for (int j = 0; j < robot_ID_dis_matrix_map[itr.first][itr.second[i]].size(); j++)
        {
          int matrix_i =
              itr.second[i];  //  matrix_i   是   外部  已探索点   在robot_ID_positions_map[robot_id]  上的索引
          //  判定   matrix_j   是否在      外部未探索点集
          int matrix_j = j;
          std::vector<int>::iterator ret =
              std::find(robot_ID_positions_ID_out_keyposegrapher[itr.first].begin(),
                        robot_ID_positions_ID_out_keyposegrapher[itr.first].end(), matrix_j);
          // std::cout << std::endl << "\033[1;32m"<<"判定是否是 在外部未探索点" << "\033[0m" << std::endl;
          if (ret != robot_ID_positions_ID_out_keyposegrapher[itr.first].end())  //  matrix_j     是外部未探索点的值
                                                                                 //  或者 是机器人位置
          {
            // std::cout << std::endl << "\033[1;32m"<<"对外部未探索点处理" << "\033[0m" << std::endl;
            // 替换   外部探索  点    matrix_i（外部已探索点）    matrix_j（外部未探索点）之间的边   并添加到  融合图中
            // Fuse_Grapher_
            double distance_ij =
                robot_ID_dis_matrix_map[itr.first][matrix_i]
                                       [matrix_j];  //   得到  与
                                                    //   robot_ID_positions_on_keyposegrapher[itr.first][j]
                                                    //   的  位置数据
            //检测  外部探索点       是不是全局探索点   若外部探索点不是全局探索点    则不必探索，不用加入到  融合图中
            geometry_msgs::msg::Point out_position;
            out_position = robot_ID_positions_map[itr.first][matrix_j];
            //由 位置数据  得到得到Cell 单元  ID   GetCellInd()
            // std::cout << std::endl << "\033[1;32m"<<"外部未探索点是否是   未探索状态" << "\033[0m" << std::endl;
            if (subspaces_->GetCell(GetCellInd(out_position.x, out_position.y, out_position.z)).GetStatus() ==
                CellStatus::EXPLORING)  //外部未探索点    是探索   或者 是机器人位置
            {
              int localexploring_nodeid = -1; 
              localexploring_nodeid = ReplaceEdge_GetLocalExploringNodeId_and_Distance(
                  robot_ID_positions_map[itr.first][matrix_i], robot_ID_positions_map[itr.first][matrix_j],
                  exploring_cell_positions, distance_ij, keypose_graph);
              if (localexploring_nodeid != -1)  //若不等于-1  则找到 ，需要添加结点和边  给到  融合图
              {
                geometry_msgs::msg::Point exploring_cell_position = exploring_cell_positions[localexploring_nodeid];
                int exploring_cell_id = exploring_cell_indices[localexploring_nodeid]; 
                int exploring_cell_robot_id = 0; 
                int exploring_cell_fuse_grapher_node_id = Fuse_Grapher_.graph_.size();  
                if (Fuse_Grapher_.graph_.empty())
                {
                  exploring_cell_fuse_grapher_node_id = 0;
                }
                fuse_node new_node(exploring_cell_id, exploring_cell_robot_id, localexploring_nodeid,
                                   exploring_cell_fuse_grapher_node_id, exploring_cell_position);
                bool is_exist_exploring = false;
                is_exist_exploring = Fuse_Grapher_.is_exist_fuse_node(new_node, exploring_cell_fuse_grapher_node_id);
                if (!is_exist_exploring)  //不存在   添加
                {
                  robot_ID_is_fuse[itr.first] = true;  //  连通的  就融合成功
                  Fuse_Grapher_.AddNode(exploring_cell_id, exploring_cell_robot_id, localexploring_nodeid,
                                        exploring_cell_fuse_grapher_node_id, exploring_cell_position);
                  std::cout << "\033[1;32m"
                            << "不存在结点 添加 robot_id:  " << exploring_cell_robot_id
                            << "  地址: " << localexploring_nodeid << "\033[0m" << std::endl;
                }
                geometry_msgs::msg::Point out_keyposegrapher_position = robot_ID_positions_map[itr.first][matrix_j];
                int out_keyposegrapher_cell_id = GetCellInd(
                    out_keyposegrapher_position.x, out_keyposegrapher_position.y, out_keyposegrapher_position.z);
                int out_keyposegrapher_robot_id = itr.first;
                int out_keyposegrapher_fuse_grapher_node_id = Fuse_Grapher_.graph_.size();
                bool is_exist_out = false;
                fuse_node new_node_(out_keyposegrapher_cell_id, out_keyposegrapher_robot_id, matrix_j,
                                    out_keyposegrapher_fuse_grapher_node_id, out_keyposegrapher_position);
                is_exist_out = Fuse_Grapher_.is_exist_fuse_node(new_node_, out_keyposegrapher_fuse_grapher_node_id);
                if (!is_exist_out)  //不存在   添加
                {
                  Fuse_Grapher_.AddNodeAndEdge(out_keyposegrapher_cell_id, out_keyposegrapher_robot_id, matrix_j,
                                               out_keyposegrapher_fuse_grapher_node_id, out_keyposegrapher_position,
                                               exploring_cell_fuse_grapher_node_id, distance_ij);
                  std::cout << "\033[1;32m"
                            << "不存在结点 添加 robot_id:  " << out_keyposegrapher_robot_id << "  地址: " << matrix_j
                            << "\033[0m" << std::endl;
                  std::cout << "\033[1;32m"
                            << "不存在 边    在    " << out_keyposegrapher_fuse_grapher_node_id << "  和  "
                            << exploring_cell_fuse_grapher_node_id << " 添加边"
                            << "\033[0m" << std::endl;
                }
                //添加边
                double distance_ = distance_ij;
                if (is_exist_out)  //需要   检测边   并添加  或替换
                {
                  bool is_edge_exist =
                      Fuse_Grapher_.is_edge_exist(exploring_cell_fuse_grapher_node_id,
                                                  out_keyposegrapher_fuse_grapher_node_id, distance_);  //存在就直接替换
                  if (!is_edge_exist)                                                                   //添加边
                  {
                    Fuse_Grapher_.AddEdge(exploring_cell_fuse_grapher_node_id, out_keyposegrapher_fuse_grapher_node_id,
                                          distance_);
                    std::cout << "\033[1;32m"
                              << "不存在 边    在    " << out_keyposegrapher_fuse_grapher_node_id << "  和  "
                              << exploring_cell_fuse_grapher_node_id << " 添加边"
                              << "\033[0m" << std::endl;
                  }
                }
              }
            }
          }
        }
      }
    }
    std::cout << std::endl
              << "\033[1;32m"
              << "外部未探索点融合"
              << "\033[0m" << std::endl;
    //检测外部传入  未探索结点   与外部未探索结点    互相的边  加入图中
    for (auto itr : robot_ID_positions_ID_out_keyposegrapher)
    {
      if (robot_ID_positions_ID_on_keyposegrapher[itr.first].empty())  //若是  外部已探索点集   没有 ,  说明
                                                                       //两个机器人没有共同区域  不融合
      {
        continue;
      }
      for (int i = 0; i < itr.second.size(); i++)  //对于外部未探索结点
      {
        int matrix_i = itr.second[i];  //  matrix_i   是   外部  未探索点   在robot_ID_positions_map[robot_id]  上的索引
        geometry_msgs::msg::Point out_position_i;
        out_position_i = robot_ID_positions_map[itr.first][matrix_i];
        if (subspaces_->GetCell(GetCellInd(out_position_i.x, out_position_i.y, out_position_i.z)).GetStatus() ==
            CellStatus::EXPLORING)  //外部探索点是  探索点
        {
          for (int j = 0; j < robot_ID_dis_matrix_map[itr.first][itr.second[i]].size(); j++)  // 外部未探索结点 需要排除
          {
            //  判定   matrix_j   是否在      外部未探索点集
            int matrix_j = j;
            std::vector<int>::iterator ret =
                std::find(robot_ID_positions_ID_out_keyposegrapher[itr.first].begin(),
                          robot_ID_positions_ID_out_keyposegrapher[itr.first].end(), matrix_j);
            if (ret != robot_ID_positions_ID_out_keyposegrapher[itr.first].end() &&
                matrix_i != matrix_j)  //  matrix_j     是外部未探索点的值  且不是同一点
            {
              //加入边到  融合图中
              double distance_ij = robot_ID_dis_matrix_map[itr.first][matrix_i][matrix_j];  //两个结点的距离信息
              geometry_msgs::msg::Point out_position_j = robot_ID_positions_map[itr.first][matrix_j];
              int out_cell_id_i = GetCellInd(out_position_i.x, out_position_i.y, out_position_i.z);  //得到  单元  ID
              int out_robot_id_i = itr.first;
              int out_fuse_grapher_node_id_i = Fuse_Grapher_.graph_.size();
              if (Fuse_Grapher_.graph_.empty())
              {
                out_fuse_grapher_node_id_i = 0;
              }
              bool is_exist_out_i = false;
              fuse_node new_node_i(out_cell_id_i, out_robot_id_i, matrix_i, out_fuse_grapher_node_id_i, out_position_i);
              is_exist_out_i = Fuse_Grapher_.is_exist_fuse_node(new_node_i, out_fuse_grapher_node_id_i);
              if (!is_exist_out_i)  //不存在  添加结点
              {
                Fuse_Grapher_.AddNode(out_cell_id_i, out_robot_id_i, matrix_i, out_fuse_grapher_node_id_i,
                                      out_position_i);
                std::cout << "\033[1;32m"
                          << "不应该添加，在前面应该已经添加了  "
                          << "\033[0m" << std::endl;
                std::cout << "\033[1;32m"
                          << "不存在结点 添加 robot_id:  " << out_robot_id_i << "  地址: " << matrix_i << "\033[0m"
                          << std::endl;
              }
              int out_cell_id_j = GetCellInd(out_position_j.x, out_position_j.y, out_position_j.z);  //得到  单元  ID
              int out_robot_id_j = itr.first;
              int out_fuse_grapher_node_id_j = Fuse_Grapher_.graph_.size();
              if (Fuse_Grapher_.graph_.empty())
              {
                out_fuse_grapher_node_id_j = 0;
              }
              bool is_exist_out_j = false;
              fuse_node new_node_j(out_cell_id_j, out_robot_id_j, matrix_j, out_fuse_grapher_node_id_j, out_position_j);
              is_exist_out_j = Fuse_Grapher_.is_exist_fuse_node(new_node_j, out_fuse_grapher_node_id_j);
              if (!is_exist_out_j)  //不存在  添加结点
              {
                Fuse_Grapher_.AddNode(out_cell_id_j, out_robot_id_j, matrix_j, out_fuse_grapher_node_id_j,
                                      out_position_j);
                std::cout << "\033[1;32m"
                          << "不应该添加，在前面应该已经添加了  "
                          << "\033[0m" << std::endl;
                std::cout << "\033[1;32m"
                          << "不存在结点 添加 robot_id:  " << out_robot_id_j << "  地址: " << matrix_j << "\033[0m"
                          << std::endl;
              }
              double distance_ = distance_ij;
              bool is_edge_exist = Fuse_Grapher_.is_edge_exist(out_fuse_grapher_node_id_i, out_fuse_grapher_node_id_j,
                                                               distance_);  //存在就直接替换
              if (!is_edge_exist)                                           //添加边
              {
                Fuse_Grapher_.AddEdge(out_fuse_grapher_node_id_i, out_fuse_grapher_node_id_j, distance_);
                std::cout << "\033[1;32m"
                          << "不存在 边    在    " << out_fuse_grapher_node_id_i << "  和  "
                          << out_fuse_grapher_node_id_j << " 添加边"
                          << "\033[0m" << std::endl;
              }
            }
          }
        }
      }
    }
    std::cout << std::endl
              << "\033[1;32m"
              << "本地探索点融合"
              << "\033[0m" << std::endl;
    //检测本地探索点 及边   加入图中
    for (int i = 0; i < exploring_cell_positions.size(); i++)
    {
      if (subspaces_->GetCell(exploring_cell_indices[i]).GetStatus() == CellStatus::EXPLORING)
      {
        for (int j = 0; j < exploring_cell_positions.size(); j++)
        {
          if (i = j)
          {
            continue;
          }
          //其中  必须得是探索结点  或者作为选取的        外部已探索结点转移的  探索点
          geometry_msgs::msg::Point exploring_position_j = exploring_cell_positions[j];
          int exploring_cell_id_j = exploring_cell_indices[j];  //得到  单元  ID
          int exploring_robot_id_j = 0;                         //本地机器人  ID   0   只是为了区分
          int exploring_fuse_grapher_node_id_j = Fuse_Grapher_.graph_.size();
          if (Fuse_Grapher_.graph_.empty())
          {
            exploring_fuse_grapher_node_id_j = 0;
          }
          fuse_node new_node_j(exploring_cell_id_j, exploring_robot_id_j, j, exploring_fuse_grapher_node_id_j,
                               exploring_position_j);
          bool is_exist_exploring_j = Fuse_Grapher_.is_exist_fuse_node(new_node_j, exploring_fuse_grapher_node_id_j);
          if ((subspaces_->GetCell(exploring_cell_indices[j]).GetStatus() == CellStatus::EXPLORING ||
               is_exist_exploring_j) &&
              i != j)  // j  是探索结点  或  在结点内存在(外部已探索 转移选择的  本地探索结点)  且 i和j 不是同一个点
          {
            if (!is_exist_exploring_j)  //添加结点 j
            {
              Fuse_Grapher_.AddNode(exploring_cell_id_j, exploring_robot_id_j, j, exploring_fuse_grapher_node_id_j,
                                    exploring_position_j);
              std::cout << "\033[1;32m"
                        << "不存在结点 添加 robot_id:  " << exploring_robot_id_j << "  地址: " << j << "\033[0m"
                        << std::endl;
            }
            //检测  结点 i
            geometry_msgs::msg::Point exploring_position_i = exploring_cell_positions[i];
            int exploring_cell_id_i = exploring_cell_indices[i];  //得到  单元  ID
            int exploring_robot_id_i = 0;                         //本地机器人  ID   0   只是为了区分
            int exploring_fuse_grapher_node_id_i = Fuse_Grapher_.graph_.size();
            if (Fuse_Grapher_.graph_.empty())
            {
              exploring_fuse_grapher_node_id_i = 0;
            }
            fuse_node new_node_i(exploring_cell_id_i, exploring_robot_id_i, i, exploring_fuse_grapher_node_id_i,
                                 exploring_position_i);
            bool is_exist_exploring_i = Fuse_Grapher_.is_exist_fuse_node(new_node_i, exploring_fuse_grapher_node_id_i);
            if (!is_exist_exploring_i)  //添加结点 j
            {
              Fuse_Grapher_.AddNode(exploring_cell_id_i, exploring_robot_id_i, i, exploring_fuse_grapher_node_id_i,
                                    exploring_position_i);
              std::cout << "\033[1;32m"
                        << "不存在结点 添加 robot_id:  " << exploring_robot_id_j << "  地址: " << j << "\033[0m"
                        << std::endl;
            }
            //检测  添加边
            double distance_ij = (double)distance_matrix[i][j];
            bool is_edge_exist = Fuse_Grapher_.is_edge_exist(
                exploring_fuse_grapher_node_id_i, exploring_fuse_grapher_node_id_j, distance_ij);  //存在就直接替换
            if (!is_edge_exist)                                                                    //添加边
            {
              Fuse_Grapher_.AddEdge(exploring_fuse_grapher_node_id_i, exploring_fuse_grapher_node_id_j, distance_ij);
              std::cout << "\033[1;32m"
                        << "不存在 边    在    " << exploring_fuse_grapher_node_id_i << "  和  "
                        << exploring_fuse_grapher_node_id_j << " 添加边"
                        << "\033[0m" << std::endl;
            }
          }
        }
      }
    }

    //  判定    机器人各自起点   ,并加入到图中         多个机器人位置     就是  子图的最后一个数据
    std::map<int, int> robot_ID_robotposetionfusegrapher_id;  //  机器人ID 和 机器人在   融合图上的   编号    融合成功的
    std::cout << std::endl
              << "\033[1;32m"
              << "外部机器人位置"
              << "\033[0m" << std::endl;
    //对于外部传来子图的机器人位置
    for (auto itr : robot_ID_positions_map)
    {
      if (robot_ID_is_fuse[itr.first])  //有相邻并融合成功   就找到机器人位置  加入 到 融合图中
      {
        int id_onrobot_ID_positions_map = itr.second.size() - 1;
        geometry_msgs::msg::Point robot_position = itr.second.back();  //编号   itr.second.size()-1
        int robot_position_cell_id = GetCellInd(robot_position.x, robot_position.y, robot_position.z);  //得到  单元  ID
        int robot_id = itr.first;
        int robot_position_fuse_grapher_node_id = Fuse_Grapher_.graph_.size();
        fuse_node new_node_(robot_position_cell_id, robot_id, id_onrobot_ID_positions_map,
                            robot_position_fuse_grapher_node_id, robot_position);
        //查询  机器人位置结点是否存在
        bool is_exist = Fuse_Grapher_.is_exist_fuse_node(new_node_, robot_position_fuse_grapher_node_id);
        if (!is_exist)  //不存在就添加
        {
          Fuse_Grapher_.AddNode(robot_position_cell_id, robot_id, id_onrobot_ID_positions_map,
                                robot_position_fuse_grapher_node_id, robot_position);
          std::cout << "\033[1;32m"
                    << "不存在机器人结点 添加 robot_id:  " << robot_id << "  地址: " << id_onrobot_ID_positions_map
                    << "\033[0m" << std::endl;
        }
        Fuse_Grapher_.fuse_nodes_[robot_position_fuse_grapher_node_id].is_robotpose = true;
        //查询融合图中  对应机器人ID  在融合图上的其他结点   连接。
        for (int i = 0; i < Fuse_Grapher_.fuse_nodes_.size(); i++)
        {
          if (Fuse_Grapher_.fuse_nodes_[i].robot_id_ == itr.first &&
              i != robot_position_fuse_grapher_node_id)  //查询     对应机器人ID在融合图上的其他结点
          {
            //得到两个结点的   距离
            int matrix_i = Fuse_Grapher_.fuse_nodes_[i].positions_id_;
            int fuse_grapher_node_id_i = Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_;
            double distance_ = (double)robot_ID_dis_matrix_map[itr.first][id_onrobot_ID_positions_map][matrix_i];
            //查询是否存在边   存在就改正，不存在就添加
            bool is_edge_exist = Fuse_Grapher_.is_edge_exist(robot_position_fuse_grapher_node_id,
                                                             fuse_grapher_node_id_i, distance_);  //存在就直接替换
            if (!is_edge_exist)                                                                   //添加边
            {
              Fuse_Grapher_.AddEdge(robot_position_fuse_grapher_node_id, fuse_grapher_node_id_i, distance_);
              std::cout << "\033[1;32m"
                        << "不存在 与其他机器人相连 边    在    " << robot_position_fuse_grapher_node_id << "  和  "
                        << fuse_grapher_node_id_i << " 添加边"
                        << "\033[0m" << std::endl;
            }
          }
        }
        //得到  融合成功    的机器人  在  融合图上的 编号
        robot_ID_robotposetionfusegrapher_id.insert(
            std::pair<int, int>(itr.first, robot_position_fuse_grapher_node_id));
      }
    }
    // std::cout << std::endl << "\033[1;32m" << "最终融合" << "\033[0m" << std::endl;
    // 判定是否    融合成功
    if (!robot_ID_robotposetionfusegrapher_id.empty())  //若不为空     则融合成功
    {
      //加入本地机器人位置到  融合图中
      int id_robot_exploring = exploring_cell_positions.size() - 1;
      geometry_msgs::msg::Point robot_position = exploring_cell_positions.back();    //编号   itr.second.size()-1
      int robot_position_cell_id = exploring_cell_indices[id_robot_exploring];  //得到  单元  ID
      int robot_id = 0;                                                         //机器人ID
      int robot_position_fuse_grapher_node_id = Fuse_Grapher_.graph_.size();
      fuse_node new_node_(robot_position_cell_id, robot_id, id_robot_exploring, robot_position_fuse_grapher_node_id,
                          robot_position);
      //查询  机器人位置结点是否存在
      bool is_exist = Fuse_Grapher_.is_exist_fuse_node(new_node_, robot_position_fuse_grapher_node_id);
      if (!is_exist)  //不存在就添加
      {
        Fuse_Grapher_.AddNode(robot_position_cell_id, robot_id, id_robot_exploring, robot_position_fuse_grapher_node_id,
                              robot_position);
        std::cout << "\033[1;32m"
                  << "不存在机器人结点 添加 robot_id:  " << robot_id << "  地址: " << id_robot_exploring << "\033[0m"
                  << std::endl;
      }
      Fuse_Grapher_.fuse_nodes_[robot_position_fuse_grapher_node_id].is_robotpose = true;
      //查询在融合图上的    来自相同子图的  边
      for (int i = 0; i < Fuse_Grapher_.fuse_nodes_.size(); i++)
      {
        if (Fuse_Grapher_.fuse_nodes_[i].robot_id_ == robot_id &&
            i != robot_position_fuse_grapher_node_id)  //查询     对应机器人ID在  融合图上的其他结点
        {
          //得到两个结点的   距离
          int matrix_i = Fuse_Grapher_.fuse_nodes_[i].positions_id_;
          int fuse_grapher_node_id_i = Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_;
          double distance_ = (double)distance_matrix[id_robot_exploring][matrix_i];
          //查询是否存在边   存在就改正，不存在就添加
          bool is_edge_exist = Fuse_Grapher_.is_edge_exist(robot_position_fuse_grapher_node_id, fuse_grapher_node_id_i,
                                                           distance_);  //存在就直接替换
          if (!is_edge_exist)                                           //添加边
          {
            Fuse_Grapher_.AddEdge(robot_position_fuse_grapher_node_id, fuse_grapher_node_id_i, distance_);
            std::cout << "\033[1;32m"
                      << "不存在 与本地机器人相连的边    在    " << robot_position_fuse_grapher_node_id << "  和  "
                      << fuse_grapher_node_id_i << " 添加边"
                      << "\033[0m" << std::endl;
          }
        }
      }
      //    根据      融合图   计算   距离矩阵distance_matrix      和  探索点集 exploring_cell_positions    探索点cell
      //    ID   exploring_cell_indices    及机器人编号   （起点）

      int fuse_grapher_nodes_num = Fuse_Grapher_.fuse_nodes_.size();            //融合图  结点数
      std::vector<geometry_msgs::msg::Point> exploring_cell_positions_fuse_grapher;  //融合图   结点位置  数据集
      std::vector<int> exploring_cell_indices_fuse_grapher;  //融合图  结点  所在 Cell  单元  编码
      std::vector<int> robot_start_id_on_exploring_cell_positions_fuse_grapher;  //  多机器人  起点
      std::vector<std::vector<int>> distance_matrix_fuse_grapher(
          fuse_grapher_nodes_num, std::vector<int>(fuse_grapher_nodes_num, 0));  //融合图的距离矩阵
      //填充  融合图 结点位置  数据集   exploring_cell_positions_fuse_grapher
      for (int i = 0; i < fuse_grapher_nodes_num; i++)
      {
        exploring_cell_positions_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].position_);
        exploring_cell_indices_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].Cell_id_);
      }
      //得到 多机器人  起点
      robot_start_id_on_exploring_cell_positions_fuse_grapher.push_back(
          robot_position_fuse_grapher_node_id);  //第一个放入  本地机器人位置
      for (auto itr : robot_ID_robotposetionfusegrapher_id)
      {
        robot_start_id_on_exploring_cell_positions_fuse_grapher.push_back(itr.second);  //放入其他机器人位置
      }
      //构建距离矩阵
      for (int i = 0; i < fuse_grapher_nodes_num; i++)
      {
        for (int j = 0; j < i; j++)
        {
          if (!use_keypose_graph_ || keypose_graph == nullptr ||
              keypose_graph->GetNodeNum() == 0)  //不使用关键位姿图   直接用两点 直线距离构造边
          {
            // Use straight line connection     初始化  使用直线连接
            distance_matrix_fuse_grapher[i][j] = static_cast<int>(
                10 * misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
                         exploring_cell_positions_fuse_grapher[i], exploring_cell_positions_fuse_grapher[j]));
          }
          else
          {
            // Use keypose graph   使用关键位姿图
            nav_msgs::msg::Path path_tmp;
            distance_matrix_fuse_grapher[i][j] = static_cast<int>(
                10 * GetShortestPath_Fuse_Grapher(Fuse_Grapher_, i, j));  //在 关键位姿图上   找到最短距离
          }
        }
      }
      //对角化
      for (int i = 0; i < fuse_grapher_nodes_num; i++)
      {
        for (int j = i + 1; j < fuse_grapher_nodes_num; j++)
        {
          distance_matrix_fuse_grapher[i][j] = distance_matrix_fuse_grapher[j][i];
        }
      }

      std::cout << std::endl
                << "\033[1;32m"
                << "成功"
                << "\033[0m" << std::endl;
      //赋值   传出
      exploring_cell_indices_MTSP = exploring_cell_indices_fuse_grapher;
      std::cout << "\033[1;32m"
                << "融合图结点单元 Cell _ID: "
                << "\033[0m" << std::endl;
      for (int j = 0; j < exploring_cell_indices_fuse_grapher.size(); j++)
      {
        std::cout << "\033[1;32m" << exploring_cell_indices_fuse_grapher[j] << "\033[0m" << std::endl;
      }
      exploring_cell_positions_MTSP = exploring_cell_positions_fuse_grapher;
      distance_matrix_MTSP = distance_matrix_fuse_grapher;
      robot_position_id_on_exploring_cell_positions_MTSP = robot_start_id_on_exploring_cell_positions_fuse_grapher;
      return true;
    }
    else
    {  //融合失败   没用共同的点
      std::cout << std::endl
                << "\033[1;32m"
                << " 融合失败  没有共同点"
                << "\033[0m" << std::endl;
      return false;
    }
  }
  else  //没有外部输入子图
  {
    std::cout << std::endl
              << "\033[1;32m"
              << "   融合失败  没有传入子图"
              << "\033[0m" << std::endl;
    return false;  //融合失败    做TSP
  }
}

//  传入    外部已探索点位置， 外部未探索点位置，本地探索点集，返回本地探索点集ID   返回  -1  就是找不到
int GridWorld::ReplaceEdge_GetLocalExploringNodeId_and_Distance(
    geometry_msgs::msg::Point& on_keyposegrapher,                 //外部已探索点
    geometry_msgs::msg::Point& out_keyposegrapher,                //外部传入未探索点
    std::vector<geometry_msgs::msg::Point>& exploring_positions,  //本地探索点
    double& distance_,
    const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)  //两点距离
{
  int dis_ = (int)distance_;
  int dis = 0;
  bool is_found = false;
  int exploring_positions_id = -1;
  for (int i = 0; i < exploring_positions.size(); i++)
  {
    int dis_onkeyposegrapher_exploring_position = INT_MAX / 2;   //外部已探索点  和本地探索点的距离
    int dis_outkeyposegrapher_exploring_position = INT_MAX / 2;  //外部未探索点  和本地探索点的距离
    //计算  on_keyposegrapher 与 exploring_positions[i]在  关键位姿图上的最短路径
    if (!use_keypose_graph_ || keypose_graph == nullptr ||
        keypose_graph->GetNodeNum() == 0)  //不使用关键位姿图   直接用两点 直线距离构造边
    {
      // Use straight line connection     初始化  使用直线连接
      dis_onkeyposegrapher_exploring_position =
          static_cast<int>(10 * misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
                                    on_keyposegrapher, exploring_positions[i]));
    }
    else
    {
      // Use keypose graph   使用关键位姿图
      nav_msgs::msg::Path path_tmp;
      dis_onkeyposegrapher_exploring_position =
          static_cast<int>(10 * keypose_graph->GetShortestPath(on_keyposegrapher, exploring_positions[i], false,
                                                               path_tmp, false));  //在 关键位姿图上   找到最短距离
    }
    //计算  本地探索点   和    外部探索点  距离
    dis_outkeyposegrapher_exploring_position =
        static_cast<int>(10 * misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
                                  out_keyposegrapher, exploring_positions[i]));
    if ((dis_outkeyposegrapher_exploring_position + dis_onkeyposegrapher_exploring_position) < dis_)  //越小越好
    {
      is_found = true;
      dis_ = dis_outkeyposegrapher_exploring_position + dis_onkeyposegrapher_exploring_position;
      dis = (int)distance_ - dis_onkeyposegrapher_exploring_position;  //距离等于s-l
      exploring_positions_id = i;
    }
  }
  //
  if (is_found)
  {
    distance_ = (double)dis;
    return exploring_positions_id;
  }
  else
  {
    return exploring_positions_id;
  }
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
// 计算  返回路径
std::vector<int> GridWorld::GetShortestPath_Fuse_Grapher_pro(fuse_grapher& Fuse_Graoher_, int start_id, int target_id)
{
  std::vector<int> path_indices;
  if (Fuse_Graoher_.fuse_nodes_.size() < 2)
  {
    std::cout << "融合图点个数" << std::endl;
    return path_indices;  //返回空
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
  double shortest_dist = misc_utils_ns::AStarSearch(Fuse_Graoher_.graph_, Fuse_Graoher_.dist_, node_positions, from_idx,
                                                    to_idx, true, path_indices);
  std::cout << "融合图  id  " << from_idx << " 和 "
            << "to_idx   "
            << " 长度   " << shortest_dist << " 路径长度  " << path_indices.size() << std::endl;
  for (int i = 0; i < path_indices.size(); i++)
  {
    std::cout << path_indices[i] << "  ->";
  }
  std::cout << std::endl;
  return path_indices;
}

//  2代    融合局部子图  MTSP
bool GridWorld::GetGlobalMtspGrapher_FusedGrapher(
    std::vector<int>& exploring_cell_indices, std::vector<geometry_msgs::msg::Point>& exploring_cell_positions,
    std::vector<std::vector<int>>& distance_matrix,
    std::vector<int>& exploring_cell_indices_MTSP,                     //   做MTSP  的  探索单元  ID
    std::vector<geometry_msgs::msg::Point>& exploring_cell_positions_MTSP,  //做 MTSP  探索单元  ID
    std::vector<std::vector<int>>& distance_matrix_MTSP,               //  做MTSP 的距离矩阵
    std::vector<int>& robot_position_id_on_exploring_cell_positions_MTSP,
    std::map<int, std::vector<double>>& MTSP_subgrapher_map_,
    const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph, fuse_grapher Fuse_Grapher_)
{
  //解码     MTSP_subgrapher_map_
  std::map<int, std::vector<geometry_msgs::msg::Point>> robot_ID_positions_map;
  std::map<int, std::vector<std::vector<int>>> robot_ID_dis_matrix_map;
  std::map<int, std::vector<geometry_msgs::msg::Point>> robot_ID_positions_on_keyposegrapher;  //外部探索点   在本地可到达
                                                                                          //已探索点
  std::map<int, std::vector<int>>
      robot_ID_positions_ID_on_keyposegrapher; 
  std::map<int, std::vector<geometry_msgs::msg::Point>> robot_ID_positions_out_keyposegrapher;  //外部探索点   在本地不可到达
                                                                                           //未知探索点
  std::map<int, std::vector<int>>
      robot_ID_positions_ID_out_keyposegrapher;  
  std::map<int, std::vector<std::vector<int>>> Cell_ID_positions_global;  //用于做  MTSP的  单元  ID 和位置。
  std::map<int, bool> robot_ID_is_fuse;                                   //<机器人ID ， 是否融合成功 >
  int other_submtspgrapher_num;
  other_submtspgrapher_num = MTSP_subgrapher_map_.size();
  if (other_submtspgrapher_num > 0)  //当有外部输入  子图
  {
    for (auto itr : MTSP_subgrapher_map_)
    {
      robot_ID_is_fuse.insert(std::pair<int, bool>(itr.first, false));  //对每个外部子图都创建一个判定  初始化为   false
      int robot_id;
      robot_id = itr.first;
      int node_num = itr.second[0];  // 0  位是结点数量
      int count = 1;
      std::vector<geometry_msgs::msg::Point> positions;
      std::vector<std::vector<int>> dis_matrix(node_num, std::vector<int>(node_num, 0));

      geometry_msgs::msg::Point point_;
      for (int i = 1; i < itr.second.size(); i++)  //  3*n    是机器人位置信息   n*n  是距离矩阵
      {
        if (i <= 3 * node_num)  // 3*n
        {
          if (i % 3 == 1)
          {
            point_.x = itr.second[i];
          }
          else if (i % 3 == 2)
          {
            point_.y = itr.second[i];
          }
          else if (i % 3 == 0)
          {
            point_.z = itr.second[i];
            positions.push_back(point_);
          }
        }
        else
        {  //  构造   距离矩阵      n*n
          int node_i = (i - 3 * node_num - 1) / node_num;
          int node_j = (i - 3 * node_num - 1) % node_num;
          dis_matrix[node_i][node_j] = (int)itr.second[i];
        }
      }
      robot_ID_positions_map.insert(std::pair<int, std::vector<geometry_msgs::msg::Point>>(itr.first, positions));
      robot_ID_dis_matrix_map.insert(std::pair<int, std::vector<std::vector<int>>>(itr.first, dis_matrix));
    }
    for (auto itr : robot_ID_positions_map)  //   分类 目标点
    {
      std::vector<geometry_msgs::msg::Point> positions_on_keyposegrapher;   //在关键位姿图上
      std::vector<int> positions_ID_on_keyposegrapher;                 //位置索引
      std::vector<geometry_msgs::msg::Point> positions_out_keyposegrapher;  //不在关键位姿图上
      std::vector<int> positions_ID_out_keyposegrapher;                //位置索引

      //  对每一个外部       探索点
      for (int i = 0; i < itr.second.size(); i++)
      {
        int Cell_ID = GetCellInd(itr.second[i].x, itr.second[i].y, itr.second[i].z);  
        bool reachable = false;
        if (keypose_graph->IsPositionReachable(itr.second[i]))
        {
          reachable = true;
          positions_on_keyposegrapher.push_back(itr.second[i]);  //加入
          positions_ID_on_keyposegrapher.push_back(i); 
        }
        else 
        {
          double min_dist = DBL_MAX;
          double min_dist_node_ind = -1;
          for (const auto& node_ind : subspaces_->GetCell(Cell_ID).GetGraphNodeIndices())  
          {
            geometry_msgs::msg::Point node_position = keypose_graph->GetNodePosition(node_ind);  
            double dist =
                misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(node_position, itr.second[i]);
            if (dist < min_dist)  
            {
              min_dist = dist;
              min_dist_node_ind = node_ind;
            }
          }
          if (min_dist_node_ind >= 0 && min_dist_node_ind < keypose_graph->GetNodeNum())
          {
            reachable = true;
            // connection_point_geo = keypose_graph->GetNodePosition(min_dist_node_ind);
            positions_on_keyposegrapher.push_back(itr.second[i]);  //加入
            positions_ID_on_keyposegrapher.push_back(i);
          }
        }

        if (!reachable)  //不可到达，就是不在已探索地方，在外部   且外部是  探索状态 或是u  nsee
        {
          positions_out_keyposegrapher.push_back(itr.second[i]);
          positions_ID_out_keyposegrapher.push_back(i);
        }
      }
      if (positions_on_keyposegrapher.size() > 0)
      {
        robot_ID_positions_on_keyposegrapher.insert(
            std::pair<int, std::vector<geometry_msgs::msg::Point>>(itr.first, positions_on_keyposegrapher));
        robot_ID_positions_ID_on_keyposegrapher.insert(
            std::pair<int, std::vector<int>>(itr.first, positions_ID_on_keyposegrapher));
      }
      if (positions_out_keyposegrapher.size() > 0)
      {
        robot_ID_positions_out_keyposegrapher.insert(
            std::pair<int, std::vector<geometry_msgs::msg::Point>>(itr.first, positions_out_keyposegrapher));
        robot_ID_positions_ID_out_keyposegrapher.insert(
            std::pair<int, std::vector<int>>(itr.first, positions_ID_out_keyposegrapher));
      }
    }

    //打印
    for (auto itr : robot_ID_positions_map)
    {
      std::cout << "\033[1;32m"
                << "机器:" << itr.first << " 外部已探索数量："
                << robot_ID_positions_ID_on_keyposegrapher[itr.first].size() << "\033[0m" << std::endl;
      std::cout << "\033[1;32m"
                << "机器:" << itr.first << " 外部未探索数量："
                << robot_ID_positions_ID_out_keyposegrapher[itr.first].size() << "\033[0m" << std::endl;
      std::cout << "\033[1;32m"
                << "结点CEll_id  :   "
                << "\033[0m" << std::endl;
      for (int i = 0; i < itr.second.size(); i++)
      {
        int Cell_ID = GetCellInd(itr.second[i].x, itr.second[i].y, itr.second[i].z);  //得到 外部探索点对应的 单元ID
        std::cout << "\033[1;32m" << Cell_ID << "\033[0m" << std::endl;
      }
    }

    //    第一步   检测   外部已探索点个数  为0   就是没有外部已探索点  融合失败

    for (auto itr : robot_ID_positions_ID_on_keyposegrapher)
    {
      if (itr.second.empty() ||
          robot_ID_positions_ID_out_keyposegrapher[itr.first].empty())  //若是  外部已探索点集   没有 ,  说明
                                                                        //两个机器人没有共同区域  不融合   或者
                                                                        //外部未探索集没有 说明已探索完毕也不用考虑
      {
        continue;
        // std::cout << std::endl << "\033[1;32m"<<"不与机器人:" << itr.first<< " 融合"
        // <<robot_ID_positions_ID_on_keyposegrapher[itr.first].size()<< "\033[0m" << std::endl;
      }
      else
      {
        robot_ID_is_fuse[itr.first] = true;  //  连通的且有外部未探索点     就融合成功  这个机器人就得融合进来
      }
    }
    if (robot_ID_is_fuse.empty())
    {
      std::cout << "\033[1;32m"
                << "融合失败，没有满足条件的子图"
                << "\033[0m" << std::endl;
      return false;
    }
    std::cout << "\033[1;32m"
              << "MTSP本地探索点数量: " << exploring_cell_positions.size() << "\033[0m" << std::endl;

    int robot_id_ = 0;
    int robot_cell_id_ = exploring_cell_indices.back();
    int robot_position_id_ = exploring_cell_positions.size() - 1;
    geometry_msgs::msg::Point robot_position_ = exploring_cell_positions.back();  //位置数据
    int robot_fuse_grapher_node_id_ = 0;
    Fuse_Grapher_.AddNode(robot_cell_id_, robot_id_, robot_position_id_, robot_fuse_grapher_node_id_, robot_position_);

    for (auto itr : robot_ID_positions_ID_on_keyposegrapher)
    {
      if (itr.second.empty())  
      {
        continue;
      }
      for (int i = 0; i < itr.second.size(); i++)
      {
        int on_robot_id_i = itr.first;
        int on_position_id_i = itr.second[i];
        geometry_msgs::msg::Point on_position_i = robot_ID_positions_map[itr.first][on_position_id_i];
        int on_cell_id_i = GetCellInd(on_position_i.x, on_position_i.y, on_position_i.z);
        int on_fuse_grapher_node_id_i = Fuse_Grapher_.graph_.size();
        bool is_exist_on_i = false;
        fuse_node new_node_on_i(on_cell_id_i, on_robot_id_i, on_position_id_i, on_fuse_grapher_node_id_i,
                                on_position_i);
        is_exist_on_i = Fuse_Grapher_.is_exist_fuse_node(new_node_on_i, on_fuse_grapher_node_id_i);
        if (!is_exist_on_i)  //不存在  添加结点
        {
          // std::cout << "\033[1;32m"<<"不存在外部传入  结点 添加 robot_id:  "<< on_robot_id_i<<"  地址:
          // "<<on_position_id_i<<"融合图上编码： "<< on_fuse_grapher_node_id_i<< "\033[0m" << std::endl;
          Fuse_Grapher_.AddNode(on_cell_id_i, on_robot_id_i, on_position_id_i, on_fuse_grapher_node_id_i,
                                on_position_i);
        }

        //外部已探索点  和  当前机器人位置   之间的边
        nav_msgs::msg::Path path_tmp;
        double distance_ = (10 * keypose_graph->GetShortestPath(robot_position_, on_position_i, false, path_tmp,
                                                                false));  //在 关键位姿图上   找到最短距离
        bool is_edge_exist = Fuse_Grapher_.is_edge_exist(robot_fuse_grapher_node_id_, on_fuse_grapher_node_id_i,
                                                         distance_);  //存在就直接替换
        if (!is_edge_exist)                                           //添加边
        {
          // std::cout << "\033[1;32m"<<"不存在本地之间的 边    在    "<< robot_fuse_grapher_node_id_<<"  和
          // "<<on_fuse_grapher_node_id_i<<" 添加边"<< "\033[0m" << std::endl;
          Fuse_Grapher_.AddEdge(robot_fuse_grapher_node_id_, on_fuse_grapher_node_id_i, distance_);
        }

        //外部已探索点之间
        for (int j = 0; j < robot_ID_dis_matrix_map[itr.first][on_position_id_i].size(); j++)
        {
          if (on_position_id_i == j)
          {
            continue;
          }
          int robot_id_j = itr.first;
          int position_id_j = j;
          geometry_msgs::msg::Point position_j = robot_ID_positions_map[itr.first][position_id_j];
          int cell_id_j = GetCellInd(position_j.x, position_j.y, position_j.z);
          int fuse_grapher_node_id_j = Fuse_Grapher_.graph_.size();
          bool is_exist_j = false;
          fuse_node new_node_j(cell_id_j, robot_id_j, position_id_j, fuse_grapher_node_id_j, position_j);
          is_exist_j = Fuse_Grapher_.is_exist_fuse_node(new_node_j, fuse_grapher_node_id_j);
          if (!is_exist_j)  
          {
            Fuse_Grapher_.AddNode(cell_id_j, robot_id_j, position_id_j, fuse_grapher_node_id_j, position_j);
          }
          double distance_ij = robot_ID_dis_matrix_map[itr.first][on_position_id_i][j];  //两个结点的距离信息
          bool is_edge_exist = Fuse_Grapher_.is_edge_exist(on_fuse_grapher_node_id_i, fuse_grapher_node_id_j,
                                                           distance_ij);  //存在就直接替换
          if (!is_edge_exist)                                             //添加边
          {
            Fuse_Grapher_.AddEdge(on_fuse_grapher_node_id_i, fuse_grapher_node_id_j, distance_ij);
          }
        }
      }
    }

    //第二步  检测  除当前位置的    本地探索点  并加入图中
    for (int i = 0; i < exploring_cell_positions.size() - 1; i++)  //本地探索点   除开     本地机器人位置
    {
      //检测  结点 i
      geometry_msgs::msg::Point exploring_position_i = exploring_cell_positions[i];  //位置数据
      int exploring_cell_id_i = exploring_cell_indices[i];                      //得到  单元  ID
      int exploring_robot_id_i = 0;  //本地机器人  ID   0   只是为了区分
      int exploring_fuse_grapher_node_id_i = Fuse_Grapher_.graph_.size();
      if (Fuse_Grapher_.graph_.empty())
      {
        exploring_fuse_grapher_node_id_i = 0;  //融合图上的编号
      }
      fuse_node new_node_i(exploring_cell_id_i, exploring_robot_id_i, i, exploring_fuse_grapher_node_id_i,
                           exploring_position_i);
      bool is_exist_exploring_i =
          Fuse_Grapher_.is_exist_fuse_node(new_node_i, exploring_fuse_grapher_node_id_i);  // i  点是否存在
      if (!is_exist_exploring_i)                                                           //添加结点 j
      {
        // std::cout << "\033[1;32m"<<"不存在结点 添加 robot_id:  "<< exploring_robot_id_i<<"  地址: "<<
        // i<<"融合图上编码： "<< exploring_fuse_grapher_node_id_i<< "\033[0m" << std::endl;
        Fuse_Grapher_.AddNode(exploring_cell_id_i, exploring_robot_id_i, i, exploring_fuse_grapher_node_id_i,
                              exploring_position_i);
      }
      //检测本地探索点   j     及    i 与j   之间的边
      for (int j = 0; j < exploring_cell_positions.size(); j++)
      {
        if (i == j)
        {
          continue;
        }
        //检测结点 j
        geometry_msgs::msg::Point exploring_position_j = exploring_cell_positions[j];
        int exploring_cell_id_j = exploring_cell_indices[j];  //得到  单元  ID
        int exploring_robot_id_j = 0;                         //本地机器人  ID   0   只是为了区分
        int exploring_fuse_grapher_node_id_j = Fuse_Grapher_.graph_.size();
        if (Fuse_Grapher_.graph_.empty())
        {
          exploring_fuse_grapher_node_id_j = 0;
        }
        fuse_node new_node_j(exploring_cell_id_j, exploring_robot_id_j, j, exploring_fuse_grapher_node_id_j,
                             exploring_position_j);
        bool is_exist_exploring_j = Fuse_Grapher_.is_exist_fuse_node(new_node_j, exploring_fuse_grapher_node_id_j);
        if (!is_exist_exploring_j)  //添加结点 j
        {
          //  std::cout << "\033[1;32m"<<"不存在结点 添加 robot_id:  "<< exploring_robot_id_j<<"  地址: "<<
          //  j<<"融合图上编码： "<<exploring_fuse_grapher_node_id_j<< "\033[0m" << std::endl;
          Fuse_Grapher_.AddNode(exploring_cell_id_j, exploring_robot_id_j, j, exploring_fuse_grapher_node_id_j,
                                exploring_position_j);
        }
        //检测  i,j  的边   这个边是从本地距离矩阵上得到
        double distance_ij = distance_matrix[i][j];  //两个结点的距离信息
        bool is_edge_exist = Fuse_Grapher_.is_edge_exist(
            exploring_fuse_grapher_node_id_i, exploring_fuse_grapher_node_id_j, distance_ij);  //存在就直接替换
        if (!is_edge_exist)                                                                    //添加边
        {
          // std::cout << "\033[1;32m"<<"不存在本地之间的 边    在    "<< exploring_fuse_grapher_node_id_i<<"  和
          // "<<exploring_fuse_grapher_node_id_j<<" 添加边"<< "\033[0m" << std::endl;
          Fuse_Grapher_.AddEdge(exploring_fuse_grapher_node_id_i, exploring_fuse_grapher_node_id_j, distance_ij);
        }
      }

      //检测外部探索点 j    及i与j 之间的边
      for (auto itr : robot_ID_positions_ID_out_keyposegrapher)
      {
        if (robot_ID_is_fuse[itr.first] == false)  //融合失败就跳过
        {
          continue;
        }
        for (int j; j < itr.second.size(); j++)
        {
          //检测外部未探索    结点 j
          int matrix_j = itr.second[j];  // matrix_j  是  j在robot_ID_positions_map[itr.first]  上的编号索引
          geometry_msgs::msg::Point out_position_j = robot_ID_positions_map[itr.first][matrix_j];
          int out_cell_id_j = GetCellInd(out_position_j.x, out_position_j.y, out_position_j.z);  //得到  单元  ID
          int out_robot_id_j = itr.first;
          int out_fuse_grapher_node_id_j = Fuse_Grapher_.graph_.size();
          if (Fuse_Grapher_.graph_.empty())
          {
            out_fuse_grapher_node_id_j = 0;
          }
          bool is_exist_out_j = false;
          fuse_node new_node_j(out_cell_id_j, out_robot_id_j, matrix_j, out_fuse_grapher_node_id_j, out_position_j);
          is_exist_out_j = Fuse_Grapher_.is_exist_fuse_node(new_node_j, out_fuse_grapher_node_id_j);
          if (!is_exist_out_j)  //不存在  添加结点
          {
            // std::cout << "\033[1;32m"<<"不存在外部传入  结点 添加 robot_id:  "<< out_robot_id_j<<"  地址: "<<
            // matrix_j<<"融合图上编码： "<< out_fuse_grapher_node_id_j<< "\033[0m" << std::endl;
            Fuse_Grapher_.AddNode(out_cell_id_j, out_robot_id_j, matrix_j, out_fuse_grapher_node_id_j, out_position_j);
          }
          //检测  本地探索结点  与外部未探索结点  之间的边           边长度就是直线距离
          double distance_ij = 10 * misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
                                        exploring_position_i, out_position_j);
          bool is_edge_exist = Fuse_Grapher_.is_edge_exist(exploring_fuse_grapher_node_id_i, out_fuse_grapher_node_id_j,
                                                           distance_ij);  //存在就直接替换
          if (!is_edge_exist)                                             //添加边
          {
            // std::cout << "\033[1;32m"<<"不存在本地点与外部点的 边    在    "<< exploring_fuse_grapher_node_id_i<<" 和
            // "<<out_fuse_grapher_node_id_j  <<" 添加边"<< "\033[0m" << std::endl;
            Fuse_Grapher_.AddEdge(exploring_fuse_grapher_node_id_i, out_fuse_grapher_node_id_j, distance_ij);
          }
        }
      }
    }
    //第三步     本地探索结点+外部探索结点      结点都存在了   添加外部   结点互相之间的边。
    for (auto itr : robot_ID_positions_ID_out_keyposegrapher)
    {
      if (robot_ID_is_fuse[itr.first] == false)  //融合失败就跳过
      {
        continue;
      }
      for (int i = 0; i < itr.second.size(); i++)
      {
        //在  融合图上查找  找到       机器人名为  irt.first,   位置编号为 irt.second[i]的 结点     在融合图上的编号
        int out_robot_id_i = itr.first;
        int out_i = itr.second[i];
        int out_fuse_grapher_node_id_i;
        bool is_exist_i = Fuse_Grapher_.FindNode_id(out_robot_id_i, out_i, out_fuse_grapher_node_id_i);
        if (!is_exist_i)  //不存在跳出
        {
          continue;
        }
        for (int j = 0; j < itr.second.size(); j++)
        {
          if (i == j)
          {
            continue;
          }
          int out_robot_id_j = itr.first;
          int out_j = itr.second[j];
          int out_fuse_grapher_node_id_j;
          bool is_exist_j = Fuse_Grapher_.FindNode_id(out_robot_id_j, out_j, out_fuse_grapher_node_id_j);
          if (!is_exist_i)  //不存在跳出
          {
            continue;
          }
          //添加边   外部未探索点 i   和外部未探索点 j 之间的边    边长度是读取
          double distance_ij = robot_ID_dis_matrix_map[itr.first][out_i][out_j];
          bool is_edge_exist = Fuse_Grapher_.is_edge_exist(out_fuse_grapher_node_id_i, out_fuse_grapher_node_id_j,
                                                           distance_ij);  //存在就直接替换
          if (!is_edge_exist)                                             //添加边
          {
            // std::cout << "\033[1;32m"<<"不存在外部点之间的 边    在    "<< out_fuse_grapher_node_id_i<<"  和
            // "<<out_fuse_grapher_node_id_j  <<" 添加边"<< "\033[0m" << std::endl;
            Fuse_Grapher_.AddEdge(out_fuse_grapher_node_id_i, out_fuse_grapher_node_id_j, distance_ij);
          }
        }
      }
    }

    //第4步         判定本地机器人位置  ，是否给本地机器人和外部未探索点直接连线   ，及判定本地机器人是否处于
    //本地探索点边界点
    geometry_msgs::msg::Point robot_position = exploring_cell_positions.back();
    int robot_cell_id = GetCellInd(robot_position.x, robot_position.y, robot_position.z);
    int robot_fuse_grapher_node_id;
    if (subspaces_->GetCell(robot_cell_id).GetStatus() ==
        CellStatus::COVERED_BY_OTHERS)  //在本地机器人位置在  别人探索的地方 就     直接连接与外部探索探索点的位置
    {
      //找到本地机器人位置结点
      int robot_id = 0;
      int robot_position_id = exploring_cell_positions.size() - 1;
      bool is_exist_out_robot = Fuse_Grapher_.FindNode_id(robot_id, robot_position_id, robot_fuse_grapher_node_id);
      if (is_exist_out_robot)
      {
        for (auto itr : robot_ID_positions_ID_out_keyposegrapher)
        {
          if (robot_ID_is_fuse[itr.first] == false)  //融合失败就跳过
          {
            continue;
          }
          for (int i = 0; i < itr.second.size(); i++)
          {
            int robot_id = itr.first;
            int out_fuse_grapher_node_id;
            bool is_exist_out = Fuse_Grapher_.FindNode_id(robot_id, itr.second[i], out_fuse_grapher_node_id);
            if (is_exist_out)
            {
              geometry_msgs::msg::Point out_position = robot_ID_positions_map[itr.first][itr.second[i]];
              double distance = 10 * misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
                                         robot_position, out_position);
              bool is_edge_exist = Fuse_Grapher_.is_edge_exist(robot_fuse_grapher_node_id, out_fuse_grapher_node_id,
                                                               distance);  //存在就直接替换
              if (!is_edge_exist)                                          //添加边
              {
                // std::cout << "\033[1;32m"<<"不存在外部点之间的 边    在    "<<robot_fuse_grapher_node_id<<"  和
                // "<<out_fuse_grapher_node_id <<" 添加边"<< "\033[0m" << std::endl;
                Fuse_Grapher_.AddEdge(robot_fuse_grapher_node_id, out_fuse_grapher_node_id, distance);
              }
            }
          }
        }
      }
    }

    std::map<int, int> robot_ID_robotposetionfusegrapher_id;  //机器人位置    在融合图上的位置信息
    //第5步      检测外部机器人位置    是属于外部探索点  还是属于内部探索点
    for (auto itr : robot_ID_positions_map)
    {
      if (robot_ID_is_fuse[itr.first] == false)  //融合失败就跳过
      {
        continue;
      }
      int out_robot_id = itr.first;
      int out_robot_position_id = itr.second.size() - 1;
      int out_robot_fuse_grapher_node_id;
      //查询  融合途中是否有
      bool is_exist_out_robot =
          Fuse_Grapher_.FindNode_id(out_robot_id, out_robot_position_id, out_robot_fuse_grapher_node_id);
      if (is_exist_out_robot)
      {
        //存在就记录当前的机器人在融合图上的编号
        robot_ID_robotposetionfusegrapher_id.insert(std::pair<int, int>(itr.first, out_robot_fuse_grapher_node_id));
      }
    }

    //第六步  挑选   外部探索点  及 本地机器人位置   做   MTSP的数据输出
    int robot_ip;
    if (!robot_ID_robotposetionfusegrapher_id.empty())  //若不为空     则融合成功
    {
      //选中   外部探索点       及      本地机器人位置    作为     MTSP的  输入
      std::vector<geometry_msgs::msg::Point> exploring_cell_positions_fuse_grapher;  //融合图   结点位置  数据集
      std::vector<int> exploring_cell_indices_fuse_grapher;  //融合图  结点  所在 Cell  单元  编码
      std::vector<int> exploring_fuse_grapher_node_id;  //融合图  结点  所在   图上的编号   为了求距离
      std::vector<int> robot_start_id_on_exploring_cell_positions_fuse_grapher;  //  多机器人  起点
      std::vector<int> robot_start_id_on_MTSP;                                   //  多机器人  起点   在
      int fuse_grapher_nodes_num = Fuse_Grapher_.fuse_nodes_.size();             //融合图  结点数
      for (int i = 0; i < fuse_grapher_nodes_num; i++)
      {
        int robot_id = Fuse_Grapher_.fuse_nodes_[i].robot_id_;
        int cell_id = Fuse_Grapher_.fuse_nodes_[i].Cell_id_;
        if (Fuse_Grapher_.fuse_nodes_[i].robot_id_ != 0)  //不是本地的结点   且  在外部探索点上  要加入MTSP  的里面
        {
          bool is_robot = false;
          for (auto itr : robot_ID_robotposetionfusegrapher_id)  //加入机器人位置
          {
            if (Fuse_Grapher_.fuse_nodes_[i].robot_id_ == itr.first &&
                Fuse_Grapher_.fuse_nodes_[i].positions_id_ ==
                    robot_ID_positions_map[itr.first].size() - 1)  // 机器人位置
            {
              is_robot = true;
              exploring_cell_positions_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].position_);
              exploring_fuse_grapher_node_id.push_back(Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_);
              exploring_cell_indices_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].Cell_id_);
              robot_start_id_on_MTSP.push_back(exploring_cell_positions_fuse_grapher.size() - 1);
            }
          }

          //  加入探索点
          std::vector<int>::iterator ret = std::find(robot_ID_positions_ID_out_keyposegrapher[robot_id].begin(),
                                                     robot_ID_positions_ID_out_keyposegrapher[robot_id].end(),
                                                     Fuse_Grapher_.fuse_nodes_[i].positions_id_);
          if (ret == robot_ID_positions_ID_out_keyposegrapher[robot_id].end())  //指向最后，则没有找到
          {
            continue;  //不是外部未探索点  则跳过这个结点
          }
          //是已探索点  就不选。
          if (subspaces_world_->GetCell(cell_id).GetStatus() == CellStatus::COVERED ||
              subspaces_world_->GetCell(cell_id).GetStatus() == CellStatus::COVERED_BY_OTHERS)
            continue;

          // bool is_robot=false;
          // for(auto itr :robot_ID_robotposetionfusegrapher_id )
          // {
          //   if(Fuse_Grapher_.fuse_nodes_[i].robot_id_==itr.first
          //   &&Fuse_Grapher_.fuse_nodes_[i].positions_id_==robot_ID_positions_map[itr.first].size()-1 ) // 机器人位置
          //   {
          //     is_robot=true;
          //     exploring_cell_positions_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].position_);
          //     exploring_fuse_grapher_node_id.push_back(Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_);
          //     exploring_cell_indices_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].Cell_id_);
          //     robot_start_id_on_MTSP.push_back(exploring_cell_positions_fuse_grapher.size()-1);
          //   }
          // }
          if (!is_robot)  //普通点
          {
            exploring_cell_positions_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].position_);
            exploring_fuse_grapher_node_id.push_back(Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_);
            exploring_cell_indices_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].Cell_id_);
          }
        }
        else if (Fuse_Grapher_.fuse_nodes_[i].robot_id_ == 0 &&
                 Fuse_Grapher_.fuse_nodes_[i].positions_id_ == exploring_cell_positions.size() - 1)
        {
          exploring_cell_positions_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].position_);
          exploring_fuse_grapher_node_id.push_back(Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_);
          exploring_cell_indices_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].Cell_id_);
          robot_ip = exploring_cell_positions_fuse_grapher.size() - 1;
        }
      }

      int MTSP_node_num = exploring_cell_positions_fuse_grapher.size();
      std::vector<std::vector<int>> distance_matrix_fuse_grapher(
          MTSP_node_num, std::vector<int>(MTSP_node_num, 0));  //融合图的距离矩阵
      //构建距离矩阵
      for (int i = 0; i < MTSP_node_num; i++)
      {
        for (int j = 0; j < i; j++)
        {
          if (!use_keypose_graph_ || keypose_graph == nullptr ||
              keypose_graph->GetNodeNum() == 0)  //不使用关键位姿图   直接用两点 直线距离构造边
          {
            // Use straight line connection     初始化  使用直线连接
            distance_matrix_fuse_grapher[i][j] = static_cast<int>(
                10 * misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
                         exploring_cell_positions_fuse_grapher[i], exploring_cell_positions_fuse_grapher[j]));
          }
          else
          {
            // Use keypose graph   使用关键位姿图
            nav_msgs::msg::Path path_tmp;
            distance_matrix_fuse_grapher[i][j] = static_cast<int>(
                10 * GetShortestPath_Fuse_Grapher(Fuse_Grapher_, exploring_fuse_grapher_node_id[i],
                                                  exploring_fuse_grapher_node_id[j]));  //在 关键位姿图上   找到最短距离
          }
        }
      }
      //对角化
      for (int i = 0; i < MTSP_node_num; i++)
      {
        for (int j = i + 1; j < MTSP_node_num; j++)
        {
          distance_matrix_fuse_grapher[i][j] = distance_matrix_fuse_grapher[j][i];
        }
      }
      std::cout << std::endl
                << "\033[1;32m"
                << "成功"
                << "\033[0m" << std::endl;
      //赋值   传出
      exploring_cell_indices_MTSP = exploring_cell_indices_fuse_grapher;
      std::cout << "\033[1;32m"
                << "融合图结点单元 Cell _ID: "
                << "\033[0m" << std::endl;
      for (int j = 0; j < exploring_cell_indices_fuse_grapher.size(); j++)
      {
        std::cout << "\033[1;32m" << exploring_cell_indices_fuse_grapher[j] << "\033[0m" << std::endl;
      }
      exploring_cell_positions_MTSP = exploring_cell_positions_fuse_grapher;
      distance_matrix_MTSP = distance_matrix_fuse_grapher;
      robot_position_id_on_exploring_cell_positions_MTSP.push_back(robot_ip);  //   本地机器人放第一个
      for (int i = 0; i < robot_start_id_on_MTSP.size(); i++)
      {
        robot_position_id_on_exploring_cell_positions_MTSP.push_back(robot_start_id_on_MTSP[i]);
      }
      return true;
    }
    else
    {  //融合失败   没用共同的点
      std::cout << std::endl
                << "\033[1;32m"
                << " 融合失败  没有共同点"
                << "\033[0m" << std::endl;
      return false;
    }
  }
  else  //没有外部输入子图
  {
    std::cout << std::endl
              << "\033[1;32m"
              << "   融合失败  没有传入子图"
              << "\033[0m" << std::endl;
    return false;  //融合失败    做TSP
  }
}

// ******************************************* 3代    融合局部子图  MTSP  ********************************************//
bool GridWorld::GetGlobalMtspGrapher_FusedGrapher_pro(
    std::vector<int>& exploring_cell_indices, std::vector<geometry_msgs::msg::Point>& exploring_cell_positions,
    std::vector<std::vector<int>>& distance_matrix,
    std::vector<int>& exploring_cell_indices_MTSP,                     //   做MTSP  的  探索单元  ID
    std::vector<geometry_msgs::msg::Point>& exploring_cell_positions_MTSP,  //做 MTSP  探索单元  ID
    std::vector<std::vector<int>>& distance_matrix_MTSP,               //  做MTSP 的距离矩阵
    std::vector<int>& robot_position_id_on_exploring_cell_positions_MTSP,
    std::map<int, std::vector<double>>& MTSP_subgrapher_map_,
    const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph, fuse_grapher& Fuse_Grapher_,
    std::vector<int>& positions_fuse_grapher_node_id)
{
  std::map<int, std::vector<geometry_msgs::msg::Point>> robot_ID_positions_map;
  std::map<int, std::vector<std::vector<int>>> robot_ID_dis_matrix_map;
  std::map<int, std::vector<geometry_msgs::msg::Point>> robot_ID_positions_on_keyposegrapher;  //外部探索点   在本地可到达
                                                                                          //已探索点
  std::map<int, std::vector<int>>
      robot_ID_positions_ID_on_keyposegrapher;  //外部探索点   在本地可到达  已探索点
                                                //<机器人编号,外部探索点在位置向量中的索引ID>
  std::map<int, std::vector<geometry_msgs::msg::Point>> robot_ID_positions_out_keyposegrapher;  //外部探索点   在本地不可到达
                                                                                           //未知探索点
  std::map<int, std::vector<int>>
      robot_ID_positions_ID_out_keyposegrapher;  //外部探索点   在本地不可到达  未知探索点
                                                 //<机器人编号,外部探索点在位置向量中的索引ID>
  std::map<int, std::vector<std::vector<int>>> Cell_ID_positions_global;  //用于做  MTSP的  单元  ID 和位置。
  std::map<int, bool> robot_ID_is_fuse;                                   //<机器人ID ， 是否融合成功 >
  int other_submtspgrapher_num;
  other_submtspgrapher_num = MTSP_subgrapher_map_.size();
  if (other_submtspgrapher_num > 0)  //当有外部输入  子图
  {
    //对每个外部输入的子图       解码   得到  robot_ID_positions_map   robot_ID_dis_matrix_map  两个字典
    // std::cout << std::endl << "\033[1;32m" << "解码生成子图字典，并存储" << "\033[0m" << std::endl;
    for (auto itr : MTSP_subgrapher_map_)
    {
      robot_ID_is_fuse.insert(std::pair<int, bool>(itr.first, false));  //对每个外部子图都创建一个判定  初始化为   false
      int robot_id;
      robot_id = itr.first;
      int node_num = itr.second[0];  // 0  位是结点数量
      int count = 1;
      std::vector<geometry_msgs::msg::Point> positions;
      std::vector<std::vector<int>> dis_matrix(node_num, std::vector<int>(node_num, 0));

      geometry_msgs::msg::Point point_;
      for (int i = 1; i < itr.second.size(); i++)  //  3*n    是机器人位置信息   n*n  是距离矩阵
      {
        if (i <= 3 * node_num)  // 3*n
        {
          if (i % 3 == 1)
          {
            point_.x = itr.second[i];
          }
          else if (i % 3 == 2)
          {
            point_.y = itr.second[i];
          }
          else if (i % 3 == 0)
          {
            point_.z = itr.second[i];
            positions.push_back(point_);
          }
        }
        else
        {  //  构造   距离矩阵      n*n
          int node_i = (i - 3 * node_num - 1) / node_num;
          int node_j = (i - 3 * node_num - 1) % node_num;
          dis_matrix[node_i][node_j] = (int)itr.second[i];
        }
      }
      robot_ID_positions_map.insert(std::pair<int, std::vector<geometry_msgs::msg::Point>>(itr.first, positions));
      robot_ID_dis_matrix_map.insert(std::pair<int, std::vector<std::vector<int>>>(itr.first, dis_matrix));
    }
    for (auto itr : robot_ID_positions_map)  //   分类 目标点
    {
      std::vector<geometry_msgs::msg::Point> positions_on_keyposegrapher;   //在关键位姿图上
      std::vector<int> positions_ID_on_keyposegrapher;                 //位置索引
      std::vector<geometry_msgs::msg::Point> positions_out_keyposegrapher;  //不在关键位姿图上
      std::vector<int> positions_ID_out_keyposegrapher;                //位置索引

      //  对每一个外部       探索点
      for (int i = 0; i < itr.second.size(); i++)
      {
        int Cell_ID = GetCellInd(itr.second[i].x, itr.second[i].y, itr.second[i].z);  //得到 外部探索点对应的 单元ID
        //  检查  对应点   是否能够连上当前的关键位姿图 上   能连       则放入能连字典   不能连  放入不能连字典。
        bool reachable = false;  //初始定义当前子空间单元不可通行
        if (keypose_graph->IsPositionReachable(itr.second[i]))
        {
          reachable = true;
          positions_on_keyposegrapher.push_back(itr.second[i]);  //加入
          positions_ID_on_keyposegrapher.push_back(i);           //  i  是  robot_ID_positions_map  中
                                                        //  vector<geometry_msgs::msg::Point>  的point  的位置索引
        }
        else  //当不可通行时寻找此单元其他关键位姿结点   存在  就相当于  探索过，能够连接
        {
          // Check all the keypose graph nodes within this cell to see if there are any connected nodes
          // 检查此单元格中的所有keypose图形节点，以查看是否存在任何连接的节点
          double min_dist = DBL_MAX;
          double min_dist_node_ind = -1;
          for (const auto& node_ind : subspaces_->GetCell(Cell_ID).GetGraphNodeIndices())  //相同  单元      存在图结点
          {
            geometry_msgs::msg::Point node_position = keypose_graph->GetNodePosition(node_ind);  //单元内  存在于图上的结点集
            double dist =
                misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(node_position, itr.second[i]);
            if (dist < min_dist)  //找到当前单元图结点上      距离  外部给入探索点  最近的距离
            {
              min_dist = dist;
              min_dist_node_ind = node_ind;
            }
          }
          if (min_dist_node_ind >= 0 && min_dist_node_ind < keypose_graph->GetNodeNum())
          {
            reachable = true;
            // connection_point_geo = keypose_graph->GetNodePosition(min_dist_node_ind);
            positions_on_keyposegrapher.push_back(itr.second[i]);  //加入
            positions_ID_on_keyposegrapher.push_back(i);
          }
        }
        //  可以达到的判定条件  1.在keyposegrapher 中结点 距离在  0.5  内  2. 同一个单元格内存在  结点  认为可以达到
        if (!reachable)  //不可到达，就是不在已探索地方，在外部   且外部是  探索状态 或是u  nsee
        {
          positions_out_keyposegrapher.push_back(itr.second[i]);
          positions_ID_out_keyposegrapher.push_back(i);
        }
      }
      if (positions_on_keyposegrapher.size() > 0)
      {
        robot_ID_positions_on_keyposegrapher.insert(
            std::pair<int, std::vector<geometry_msgs::msg::Point>>(itr.first, positions_on_keyposegrapher));
        robot_ID_positions_ID_on_keyposegrapher.insert(
            std::pair<int, std::vector<int>>(itr.first, positions_ID_on_keyposegrapher));
      }
      if (positions_out_keyposegrapher.size() > 0)
      {
        robot_ID_positions_out_keyposegrapher.insert(
            std::pair<int, std::vector<geometry_msgs::msg::Point>>(itr.first, positions_out_keyposegrapher));
        robot_ID_positions_ID_out_keyposegrapher.insert(
            std::pair<int, std::vector<int>>(itr.first, positions_ID_out_keyposegrapher));
      }
    }

    //打印
    for (auto itr : robot_ID_positions_map)
    {
      std::cout << "\033[1;32m"
                << "机器:" << itr.first << " 外部已探索数量："
                << robot_ID_positions_ID_on_keyposegrapher[itr.first].size() << "\033[0m" << std::endl;
      std::cout << "\033[1;32m"
                << "机器:" << itr.first << " 外部未探索数量："
                << robot_ID_positions_ID_out_keyposegrapher[itr.first].size() << "\033[0m" << std::endl;
      std::cout << "\033[1;32m"
                << "结点CEll_id  :   "
                << "\033[0m" << std::endl;
      for (int i = 0; i < itr.second.size(); i++)
      {
        int Cell_ID = GetCellInd(itr.second[i].x, itr.second[i].y, itr.second[i].z);  //得到 外部探索点对应的 单元ID
        std::cout << "\033[1;32m" << Cell_ID << "\033[0m" << std::endl;
      }
    }

    //    检测   外部已探索点个数  为0   就是没有外部已探索点  融合失败
    for (auto itr : robot_ID_positions_ID_on_keyposegrapher)
    {
      //修改   外部以探索  固定加入起点
      // if(itr.second.empty() || robot_ID_positions_ID_out_keyposegrapher[itr.first].empty())   //若是  外部已探索点集
      // 没有 ,  说明  两个机器人没有共同区域  不融合   或者    外部未探索集没有 说明已探索完毕也不用考虑
      if (robot_ID_positions_ID_out_keyposegrapher[itr.first].empty())
      {
        continue;
        // std::cout << std::endl << "\033[1;32m"<<"不与机器人:" << itr.first<< " 融合"
        // <<robot_ID_positions_ID_on_keyposegrapher[itr.first].size()<< "\033[0m" << std::endl;
      }
      else
      {
        robot_ID_is_fuse[itr.first] = true;  //  连通的且有外部未探索点     就融合成功  这个机器人就得融合进来
      }
    }
    if (robot_ID_is_fuse.empty())
    {
      std::cout << "\033[1;32m"
                << "融合失败，没有满足条件的子图"
                << "\033[0m" << std::endl;
      return false;
    }

    std::cout << "\033[1;32m"
              << "MTSP本地探索点数量： " << exploring_cell_positions.size() << "\033[0m" << std::endl;
    //加入   当前机器人位置点
    int robot_id_ = 0;
    int robot_cell_id_ = exploring_cell_indices.back();
    int robot_position_id_ = exploring_cell_positions.size() - 1;
    geometry_msgs::msg::Point robot_position_ = exploring_cell_positions.back();  //位置数据
    int robot_fuse_grapher_node_id_ = 0;
    Fuse_Grapher_.AddNode(robot_cell_id_, robot_id_, robot_position_id_, robot_fuse_grapher_node_id_, robot_position_);
    Fuse_Grapher_.fuse_nodes_[robot_fuse_grapher_node_id_].is_Explored = true;
    Fuse_Grapher_.fuse_nodes_[robot_fuse_grapher_node_id_].is_robotpose = true;
    //加入 当前起点位置
    int robot_home_id_ = 0;
    int home_position_id_ = -1;
    geometry_msgs::msg::Point home_position_ = keypose_graph->GetFirstKeyposePosition();  //位置数据
    int home_cell_id_ = GetCellInd(home_position_.x, home_position_.y, home_position_.z);
    int home_fuse_grapher_node_id_ = 1;
    Fuse_Grapher_.AddNode(home_cell_id_, robot_home_id_, home_position_id_, home_fuse_grapher_node_id_, home_position_);
    Fuse_Grapher_.fuse_nodes_[home_fuse_grapher_node_id_].is_Explored = true;
    //加入当前起点和当前机器人位置边
    nav_msgs::msg::Path path_tmp_robot_home;
    double distance_robot_home =
        (10 * keypose_graph->GetShortestPath(robot_position_, home_position_, false, path_tmp_robot_home,
                                             false));  //在 关键位姿图上   找到最短距离
    Fuse_Grapher_.AddEdge(robot_fuse_grapher_node_id_, home_fuse_grapher_node_id_, distance_robot_home);
    std::cout << "\033[1;32m"
              << "给本地机器人    " << robot_fuse_grapher_node_id_ << "  和   home  " << home_fuse_grapher_node_id_
              << "    添加边" << distance_robot_home << "\033[0m" << std::endl;

    //把外部已探索结点加入    检测外部探索点与当前机器人位置  ，与 外部未探索点

    /*****************    屏蔽测试，用起点做      中间点  ******************/

    for (auto itr : robot_ID_positions_ID_on_keyposegrapher)  //对每个机器人
    {
      if (itr.second.empty())  //若是  外部已探索点集   没有 ,  说明  两个机器人没有共同区域  不融合
      {
        continue;
        // std::cout << std::endl << "\033[1;32m"<<"不与机器人:" << itr.first<< " 融合"
        // <<robot_ID_positions_ID_on_keyposegrapher[itr.first].size()<< "\033[0m" << std::endl;
      }
      //对其中一个机器人的   每一个  外部已探索点
      for (int i = 0; i < itr.second.size(); i++)
      {
        int on_robot_id_i = itr.first;
        int on_position_id_i = itr.second[i];
        geometry_msgs::msg::Point on_position_i = robot_ID_positions_map[itr.first][on_position_id_i];
        int on_cell_id_i = GetCellInd(on_position_i.x, on_position_i.y, on_position_i.z);
        int on_fuse_grapher_node_id_i = Fuse_Grapher_.graph_.size();
        bool is_exist_on_i = false;
        fuse_node new_node_on_i(on_cell_id_i, on_robot_id_i, on_position_id_i, on_fuse_grapher_node_id_i,
                                on_position_i);
        is_exist_on_i = Fuse_Grapher_.is_exist_fuse_node(new_node_on_i, on_fuse_grapher_node_id_i);
        if (!is_exist_on_i)  //不存在  添加结点
        {
          // std::cout << "\033[1;32m"<<"不存在外部传入  结点 添加 robot_id:  "<< on_robot_id_i<<"  地址:
          // "<<on_position_id_i<<"融合图上编码： "<< on_fuse_grapher_node_id_i<< "\033[0m" << std::endl;
          Fuse_Grapher_.AddNode(on_cell_id_i, on_robot_id_i, on_position_id_i, on_fuse_grapher_node_id_i,
                                on_position_i);
        }
        Fuse_Grapher_.fuse_nodes_[on_fuse_grapher_node_id_i].is_Explored = true;

        //外部已探索点  和  当前机器人位置   之间的边   通过  keyposegrapher  计算路径长短
        nav_msgs::msg::Path path_tmp;
        double distance_ = (10 * keypose_graph->GetShortestPath(robot_position_, on_position_i, false, path_tmp,
                                                                false));  //在 关键位姿图上   找到最短距离
        bool is_edge_exist = Fuse_Grapher_.is_edge_exist(robot_fuse_grapher_node_id_, on_fuse_grapher_node_id_i,
                                                         distance_);  //存在就直接替换
        if (!is_edge_exist)                                           //添加边
        {
          std::cout << "\033[1;32m"
                    << "给本地机器人    " << robot_fuse_grapher_node_id_ << "  和   外部已探索结点 "
                    << on_fuse_grapher_node_id_i << "    添加边" << distance_ << "\033[0m" << std::endl;
          Fuse_Grapher_.AddEdge(robot_fuse_grapher_node_id_, on_fuse_grapher_node_id_i, distance_);
        }

        //外部已探索点      与  外部已探索点、外部未探索点 之间   读取
        for (int j = 0; j < robot_ID_dis_matrix_map[itr.first][on_position_id_i].size(); j++)
        {
          if (on_position_id_i == j)
          {
            continue;
          }
          int robot_id_j = itr.first;
          int position_id_j = j;
          geometry_msgs::msg::Point position_j = robot_ID_positions_map[itr.first][position_id_j];
          int cell_id_j = GetCellInd(position_j.x, position_j.y, position_j.z);
          int fuse_grapher_node_id_j = Fuse_Grapher_.graph_.size();
          bool is_exist_j = false;
          fuse_node new_node_j(cell_id_j, robot_id_j, position_id_j, fuse_grapher_node_id_j, position_j);
          is_exist_j = Fuse_Grapher_.is_exist_fuse_node(new_node_j, fuse_grapher_node_id_j);
          if (!is_exist_j)  //不存在  添加结点
          {
            // std::cout << "\033[1;32m"<<"不存在外部传入  结点 添加 robot_id:  "<< robot_id_j<<"  地址:
            // "<<position_id_j<<"融合图上编码： "<< fuse_grapher_node_id_j<< "\033[0m" << std::endl;
            Fuse_Grapher_.AddNode(cell_id_j, robot_id_j, position_id_j, fuse_grapher_node_id_j, position_j);
          }
          //检测边   i,j
          //检测  i,j  的边   这个边是从矩阵上得到
          double distance_ij = robot_ID_dis_matrix_map[itr.first][on_position_id_i][j];  //两个结点的距离信息
          bool is_edge_exist = Fuse_Grapher_.is_edge_exist(on_fuse_grapher_node_id_i, fuse_grapher_node_id_j,
                                                           distance_ij);  //存在就直接替换
          if (!is_edge_exist)                                             //添加边
          {
            std::cout << "\033[1;32m"
                      << "给外部已探索结点    " << on_fuse_grapher_node_id_i
                      << "  和     外部结点（可能是已探索  可能是未探索）" << fuse_grapher_node_id_j << " 添加边"
                      << distance_ij << "\033[0m" << std::endl;
            Fuse_Grapher_.AddEdge(on_fuse_grapher_node_id_i, fuse_grapher_node_id_j, distance_ij);
          }
        }
      }
    }

    //第三步     本地探索结点+外部探索结点      结点都存在了   添加外部   结点互相之间的边。
    for (auto itr : robot_ID_positions_ID_out_keyposegrapher)
    {
      if (robot_ID_is_fuse[itr.first] == false)  //融合失败就跳过
      {
        continue;
      }
      for (int i = 0; i < itr.second.size(); i++)
      {
        //在  融合图上查找  找到       机器人名为  irt.first,   位置编号为 irt.second[i]的 结点     在融合图上的编号
        int out_robot_id_i = itr.first;
        int out_i = itr.second[i];
        int out_fuse_grapher_node_id_i;
        bool is_exist_i = Fuse_Grapher_.FindNode_id(out_robot_id_i, out_i, out_fuse_grapher_node_id_i);
        if (!is_exist_i)  //不存在跳出
        {
          //添加结点
          out_fuse_grapher_node_id_i = Fuse_Grapher_.graph_.size();
          geometry_msgs::msg::Point out_position_i = robot_ID_positions_map[itr.first][out_i];
          int out_cell_id_i = GetCellInd(out_position_i.x, out_position_i.y, out_position_i.z);
          Fuse_Grapher_.AddNode(out_cell_id_i, out_robot_id_i, out_i, out_fuse_grapher_node_id_i, out_position_i);
        }
        for (int j = 0; j < itr.second.size(); j++)
        {
          if (i == j)
          {
            continue;
          }
          int out_robot_id_j = itr.first;
          int out_j = itr.second[j];
          int out_fuse_grapher_node_id_j;
          bool is_exist_j = Fuse_Grapher_.FindNode_id(out_robot_id_j, out_j, out_fuse_grapher_node_id_j);
          if (!is_exist_j)  //不存在跳出
          {
            //添加结点
            out_fuse_grapher_node_id_j = Fuse_Grapher_.graph_.size();
            geometry_msgs::msg::Point out_position_j = robot_ID_positions_map[itr.first][out_j];
            int out_cell_id_j = GetCellInd(out_position_j.x, out_position_j.y, out_position_j.z);
            Fuse_Grapher_.AddNode(out_cell_id_j, out_robot_id_j, out_j, out_fuse_grapher_node_id_j, out_position_j);
          }
          //添加边   外部未探索点 i   和外部未探索点 j 之间的边    边长度是读取
          double distance_ij = robot_ID_dis_matrix_map[itr.first][out_i][out_j];
          bool is_edge_exist = Fuse_Grapher_.is_edge_exist(out_fuse_grapher_node_id_i, out_fuse_grapher_node_id_j,
                                                           distance_ij);  //存在就直接替换
          if (!is_edge_exist)                                             //添加边
          {
            std::cout << "\033[1;32m"
                      << "外部未探索结点    " << out_fuse_grapher_node_id_i << "  和   外部未探索结点  "
                      << out_fuse_grapher_node_id_j << " 添加边" << distance_ij << "\033[0m" << std::endl;
            Fuse_Grapher_.AddEdge(out_fuse_grapher_node_id_i, out_fuse_grapher_node_id_j, distance_ij);
          }
        }

        // 给外部未探索结点  和  home  添加边，这个边是用于以防万一没有  外部已探索结点时  寻求路径的
        double distance_i_home = 9999;  //输出是 int  不能超过  范围
        bool is_edge_exist = Fuse_Grapher_.is_edge_exist(out_fuse_grapher_node_id_i, home_fuse_grapher_node_id_,
                                                         distance_i_home);  //存在就直接替换
        if (!is_edge_exist)                                                 //添加边
        {
          std::cout << "\033[1;32m"
                    << "home    " << home_fuse_grapher_node_id_ << "  和   外部未探索结点  "
                    << out_fuse_grapher_node_id_i << " 添加边 " << distance_i_home << "\033[0m" << std::endl;
          Fuse_Grapher_.AddEdge(out_fuse_grapher_node_id_i, home_fuse_grapher_node_id_, distance_i_home);
        }
      }
    }

    geometry_msgs::msg::Point robot_position = exploring_cell_positions.back();
    int robot_cell_id = GetCellInd(robot_position.x, robot_position.y, robot_position.z);
    int robot_fuse_grapher_node_id;

    std::map<int, int> robot_ID_robotposetionfusegrapher_id;  //机器人位置    在融合图上的位置信息
    //第5步      检测外部机器人位置
    for (auto itr : robot_ID_positions_map)
    {
      if (robot_ID_is_fuse[itr.first] == false)  //融合失败就跳过
      {
        continue;
      }
      int out_robot_id = itr.first;
      int out_robot_position_id = itr.second.size() - 1;
      int out_robot_fuse_grapher_node_id;
      //查询  融合途中是否有
      bool is_exist_out_robot =
          Fuse_Grapher_.FindNode_id(out_robot_id, out_robot_position_id, out_robot_fuse_grapher_node_id);
      if (is_exist_out_robot)
      {
        //存在就记录当前的机器人在融合图上的编号
        robot_ID_robotposetionfusegrapher_id.insert(std::pair<int, int>(itr.first, out_robot_fuse_grapher_node_id));
        Fuse_Grapher_.fuse_nodes_[out_robot_fuse_grapher_node_id].is_robotpose = true;
      }
    }

    //第六步  挑选   外部探索点  及 本地机器人位置   做   MTSP的数据输出
    int robot_ip;
    if (!robot_ID_robotposetionfusegrapher_id.empty())  //若不为空     则融合成功
    {
      //选中   外部探索点       及      本地机器人位置    作为     MTSP的  输入
      std::vector<geometry_msgs::msg::Point> exploring_cell_positions_fuse_grapher;  //融合图   结点位置  数据集
      std::vector<int> exploring_cell_indices_fuse_grapher;  //融合图  结点  所在 Cell  单元  编码
      std::vector<int> exploring_fuse_grapher_node_id;  //融合图  结点  所在   图上的编号   为了求距离
      std::vector<int> robot_start_id_on_exploring_cell_positions_fuse_grapher;  //  多机器人  起点
      std::vector<int> robot_start_id_on_MTSP;                                   //  多机器人  起点   在
      int fuse_grapher_nodes_num = Fuse_Grapher_.fuse_nodes_.size();             //融合图  结点数
      for (int i = 0; i < fuse_grapher_nodes_num; i++)                           //对所有融合节点
      {
        int robot_id = Fuse_Grapher_.fuse_nodes_[i].robot_id_;
        int cell_id = Fuse_Grapher_.fuse_nodes_[i].Cell_id_;
        if (Fuse_Grapher_.fuse_nodes_[i].robot_id_ != 0)  //不是本地的结点   且  在外部探索点上  要加入MTSP  的里面
        {
          bool is_robot = false;
          for (auto itr : robot_ID_robotposetionfusegrapher_id)  //加入机器人位置
          {
            if (Fuse_Grapher_.fuse_nodes_[i].robot_id_ == itr.first &&
                Fuse_Grapher_.fuse_nodes_[i].positions_id_ ==
                    robot_ID_positions_map[itr.first].size() - 1)  // 机器人位置
            {
              is_robot = true;
              exploring_cell_positions_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].position_);
              exploring_fuse_grapher_node_id.push_back(Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_);
              std::cout << "机器人：" << itr.first << "   在融合图上的编号： "
                        << Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_ << std::endl;
              exploring_cell_indices_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].Cell_id_);
              robot_start_id_on_MTSP.push_back(exploring_cell_positions_fuse_grapher.size() - 1);
            }
          }

          //  加入探索点
          std::vector<int>::iterator ret = std::find(robot_ID_positions_ID_out_keyposegrapher[robot_id].begin(),
                                                     robot_ID_positions_ID_out_keyposegrapher[robot_id].end(),
                                                     Fuse_Grapher_.fuse_nodes_[i].positions_id_);
          if (ret == robot_ID_positions_ID_out_keyposegrapher[robot_id].end())  //指向最后，则没有找到
          {
            continue;  //不是外部未探索点  则跳过这个结点
          }
          //是已探索点  就不选。
          if (subspaces_world_->GetCell(cell_id).GetStatus() == CellStatus::COVERED ||
              subspaces_world_->GetCell(cell_id).GetStatus() == CellStatus::COVERED_BY_OTHERS)
            continue;
          if (!is_robot)  //普通点  外部未探索点
          {
            exploring_cell_positions_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].position_);
            exploring_fuse_grapher_node_id.push_back(Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_);
            std::cout << "机器人：" << Fuse_Grapher_.fuse_nodes_[i].robot_id_
                      << "   选中外部未探索结点  在融合图上的编号 ： "
                      << Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_ << std::endl;
            exploring_cell_indices_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].Cell_id_);
          }
        }
        else if (Fuse_Grapher_.fuse_nodes_[i].robot_id_ == 0 &&
                 Fuse_Grapher_.fuse_nodes_[i].positions_id_ == exploring_cell_positions.size() - 1)  //当前机器人点
        {
          exploring_cell_positions_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].position_);
          exploring_fuse_grapher_node_id.push_back(Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_);
          std::cout << "本地机器人    在融合图上的编号： " << Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_
                    << std::endl;
          exploring_cell_indices_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].Cell_id_);
          robot_ip = exploring_cell_positions_fuse_grapher.size() - 1;
        }
      }

      int MTSP_node_num = exploring_cell_positions_fuse_grapher.size();
      std::vector<std::vector<int>> distance_matrix_fuse_grapher(
          MTSP_node_num, std::vector<int>(MTSP_node_num, 0));  //融合图的距离矩阵
      //构建距离矩阵
      for (int i = 0; i < MTSP_node_num; i++)
      {
        for (int j = 0; j < i; j++)
        {
          if (!use_keypose_graph_ || keypose_graph == nullptr ||
              keypose_graph->GetNodeNum() == 0)  //不使用关键位姿图   直接用两点 直线距离构造边
          {
            // Use straight line connection     初始化  使用直线连接
            distance_matrix_fuse_grapher[i][j] = static_cast<int>(
                10 * misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
                         exploring_cell_positions_fuse_grapher[i], exploring_cell_positions_fuse_grapher[j]));
          }
          else
          {
            // Use keypose graph   使用关键位姿图
            nav_msgs::msg::Path path_tmp;
            distance_matrix_fuse_grapher[i][j] = static_cast<int>(
                10 * GetShortestPath_Fuse_Grapher(Fuse_Grapher_, exploring_fuse_grapher_node_id[i],
                                                  exploring_fuse_grapher_node_id[j]));  //在 关键位姿图上   找到最短距离
            std::cout << "融合图上   " << exploring_fuse_grapher_node_id[i] << "     和   "
                      << exploring_fuse_grapher_node_id[j] << "    路径长度  " << distance_matrix_fuse_grapher[i][j]
                      << std::endl;
          }
        }
      }
      //对角化
      for (int i = 0; i < MTSP_node_num; i++)
      {
        for (int j = i + 1; j < MTSP_node_num; j++)
        {
          distance_matrix_fuse_grapher[i][j] = distance_matrix_fuse_grapher[j][i];
        }
      }
      std::cout << std::endl
                << "\033[1;32m"
                << "成功"
                << "\033[0m" << std::endl;
      //赋值   传出
      exploring_cell_indices_MTSP = exploring_cell_indices_fuse_grapher;
      std::cout << "\033[1;32m"
                << "融合图结点单元 Cell _ID: "
                << "\033[0m" << std::endl;
      for (int j = 0; j < exploring_cell_indices_fuse_grapher.size(); j++)
      {
        std::cout << "\033[1;32m" << exploring_cell_indices_fuse_grapher[j] << "\033[0m" << std::endl;
      }
      exploring_cell_positions_MTSP = exploring_cell_positions_fuse_grapher;
      distance_matrix_MTSP = distance_matrix_fuse_grapher;
      robot_position_id_on_exploring_cell_positions_MTSP.push_back(robot_ip);  //   本地机器人放第一个
      positions_fuse_grapher_node_id = exploring_fuse_grapher_node_id;
      for (int i = 0; i < robot_start_id_on_MTSP.size(); i++)
      {
        robot_position_id_on_exploring_cell_positions_MTSP.push_back(robot_start_id_on_MTSP[i]);
      }
      return true;
    }
    else
    {  //融合失败   没用共同的点
      std::cout << std::endl
                << "\033[1;32m"
                << " 融合失败  没有共同点"
                << "\033[0m" << std::endl;
      return false;
    }
  }
  else  //没有外部输入子图
  {
    std::cout << std::endl
              << "\033[1;32m"
              << "   融合失败  没有传入子图"
              << "\033[0m" << std::endl;
    return false;  //融合失败    做TSP
  }
}

// ***********************************************4 代 融合局部子图 MTSP**********************************************//
bool GridWorld::GetGlobalMtspGrapher_FusedGrapher_max(
    std::vector<int>& exploring_cell_indices, std::vector<geometry_msgs::msg::Point>& exploring_cell_positions,
    std::vector<std::vector<int>>& distance_matrix,
    std::vector<int>& exploring_cell_indices_MTSP,                     //   做MTSP  的  探索单元  ID
    std::vector<geometry_msgs::msg::Point>& exploring_cell_positions_MTSP,  //做 MTSP  探索单元  ID
    std::vector<std::vector<int>>& distance_matrix_MTSP,               //  做MTSP 的距离矩阵
    std::vector<int>& robot_position_id_on_exploring_cell_positions_MTSP,
    std::map<int, std::vector<double>>& MTSP_subgrapher_map_,
    const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph, fuse_grapher& Fuse_Grapher_,
    std::vector<int>& positions_fuse_grapher_node_id)
{
  std::map<int, std::vector<geometry_msgs::msg::Point>> robot_ID_positions_map;
  std::map<int, std::vector<std::vector<int>>> robot_ID_dis_matrix_map;
  std::map<int, std::vector<geometry_msgs::msg::Point>> robot_ID_positions_on_keyposegrapher;  //外部探索点   在本地可到达
                                                                                          //已探索点
  std::map<int, std::vector<int>>
      robot_ID_positions_ID_on_keyposegrapher;  //外部探索点   在本地可到达  已探索点
                                                //<机器人编号,外部探索点在位置向量中的索引ID>
  std::map<int, std::vector<geometry_msgs::msg::Point>> robot_ID_positions_out_keyposegrapher;  //外部探索点   在本地不可到达
                                                                                           //未知探索点
  std::map<int, std::vector<int>>
      robot_ID_positions_ID_out_keyposegrapher;  //外部探索点   在本地不可到达  未知探索点
                                                 //<机器人编号,外部探索点在位置向量中的索引ID>
  std::map<int, std::vector<std::vector<int>>> Cell_ID_positions_global;  //用于做  MTSP的  单元  ID 和位置。
  std::map<int, bool> robot_ID_is_fuse;                                   //<机器人ID ， 是否融合成功 >
  int other_submtspgrapher_num;
  other_submtspgrapher_num = MTSP_subgrapher_map_.size();
  if (other_submtspgrapher_num > 0)  //当有外部输入  子图
  {
    for (auto itr : MTSP_subgrapher_map_)
    {
      robot_ID_is_fuse.insert(std::pair<int, bool>(itr.first, false));  //对每个外部子图都创建一个判定  初始化为   false
      int robot_id;
      robot_id = itr.first;
      int node_num = itr.second[0];  // 0  位是结点数量
      int count = 1;
      std::vector<geometry_msgs::msg::Point> positions;
      std::vector<std::vector<int>> dis_matrix(node_num, std::vector<int>(node_num, 0));

      geometry_msgs::msg::Point point_;
      for (int i = 1; i < itr.second.size(); i++)  //  3*n    是机器人位置信息   n*n  是距离矩阵
      {
        if (i <= 3 * node_num)  // 3*n
        {
          if (i % 3 == 1)
          {
            point_.x = itr.second[i];
          }
          else if (i % 3 == 2)
          {
            point_.y = itr.second[i];
          }
          else if (i % 3 == 0)
          {
            point_.z = itr.second[i];
            positions.push_back(point_);
          }
        }
        else
        {  //  构造   距离矩阵      n*n
          int node_i = (i - 3 * node_num - 1) / node_num;
          int node_j = (i - 3 * node_num - 1) % node_num;
          dis_matrix[node_i][node_j] = (int)itr.second[i];
        }
      }
      robot_ID_positions_map.insert(std::pair<int, std::vector<geometry_msgs::msg::Point>>(itr.first, positions));
      robot_ID_dis_matrix_map.insert(std::pair<int, std::vector<std::vector<int>>>(itr.first, dis_matrix));
    }
    for (auto itr : robot_ID_positions_map)  //   分类 目标点
    {
      //  根据位置反算单元   计算  单元里面是否存在   关键位姿图结点，  存在就是可以通行的
      //  对每一个外部       探索点
      std::vector<geometry_msgs::msg::Point> positions_on_keyposegrapher;   //在关键位姿图上
      std::vector<int> positions_ID_on_keyposegrapher;                 //位置索引
      std::vector<geometry_msgs::msg::Point> positions_out_keyposegrapher;  //不在关键位姿图上
      std::vector<int> positions_ID_out_keyposegrapher;                //位置索引

      //  对每一个外部       探索点
      for (int i = 0; i < itr.second.size(); i++)
      {
        int Cell_ID = GetCellInd(itr.second[i].x, itr.second[i].y, itr.second[i].z);  //得到 外部探索点对应的 单元ID
        //  检查  对应点   是否能够连上当前的关键位姿图 上   能连       则放入能连字典   不能连  放入不能连字典。
        bool reachable = false;  //初始定义当前子空间单元不可通行
        if (keypose_graph->IsPositionReachable(itr.second[i]))
        {
          reachable = true;
          positions_on_keyposegrapher.push_back(itr.second[i]);  //加入
          positions_ID_on_keyposegrapher.push_back(i);           //  i  是  robot_ID_positions_map  中
                                                        //  vector<geometry_msgs::msg::Point>  的point  的位置索引
        }
        else  //当不可通行时寻找此单元其他关键位姿结点   存在  就相当于  探索过，能够连接
        {
          // Check all the keypose graph nodes within this cell to see if there are any connected nodes
          // 检查此单元格中的所有keypose图形节点，以查看是否存在任何连接的节点
          double min_dist = DBL_MAX;
          double min_dist_node_ind = -1;
          for (const auto& node_ind : subspaces_->GetCell(Cell_ID).GetGraphNodeIndices())  //相同  单元      存在图结点
          {
            geometry_msgs::msg::Point node_position = keypose_graph->GetNodePosition(node_ind);  //单元内  存在于图上的结点集
            double dist =
                misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(node_position, itr.second[i]);
            if (dist < min_dist)  //找到当前单元图结点上      距离  外部给入探索点  最近的距离
            {
              min_dist = dist;
              min_dist_node_ind = node_ind;
            }
          }
          if (min_dist_node_ind >= 0 && min_dist_node_ind < keypose_graph->GetNodeNum())
          {
            reachable = true;
            // connection_point_geo = keypose_graph->GetNodePosition(min_dist_node_ind);
            positions_on_keyposegrapher.push_back(itr.second[i]);  //加入
            positions_ID_on_keyposegrapher.push_back(i);
          }
        }
        //  可以达到的判定条件  1.在keyposegrapher 中结点 距离在  0.5  内  2. 同一个单元格内存在  结点  认为可以达到
        if (!reachable)  //不可到达，就是不在已探索地方，在外部   且外部是  探索状态 或是u  nsee
        {
          positions_out_keyposegrapher.push_back(itr.second[i]);
          positions_ID_out_keyposegrapher.push_back(i);
        }
      }
      if (positions_on_keyposegrapher.size() > 0)
      {
        robot_ID_positions_on_keyposegrapher.insert(
            std::pair<int, std::vector<geometry_msgs::msg::Point>>(itr.first, positions_on_keyposegrapher));
        robot_ID_positions_ID_on_keyposegrapher.insert(
            std::pair<int, std::vector<int>>(itr.first, positions_ID_on_keyposegrapher));
      }
      if (positions_out_keyposegrapher.size() > 0)
      {
        robot_ID_positions_out_keyposegrapher.insert(
            std::pair<int, std::vector<geometry_msgs::msg::Point>>(itr.first, positions_out_keyposegrapher));
        robot_ID_positions_ID_out_keyposegrapher.insert(
            std::pair<int, std::vector<int>>(itr.first, positions_ID_out_keyposegrapher));
      }
    }

    //打印
    for (auto itr : robot_ID_positions_map)
    {
      std::cout << "\033[1;32m"
                << "机器:" << itr.first << " 外部已探索数量："
                << robot_ID_positions_ID_on_keyposegrapher[itr.first].size() << "\033[0m" << std::endl;
      std::cout << "\033[1;32m"
                << "机器:" << itr.first << " 外部未探索数量："
                << robot_ID_positions_ID_out_keyposegrapher[itr.first].size() << "\033[0m" << std::endl;
      std::cout << "\033[1;32m"
                << "结点CEll_id  :   "
                << "\033[0m" << std::endl;
      for (int i = 0; i < itr.second.size(); i++)
      {
        int Cell_ID = GetCellInd(itr.second[i].x, itr.second[i].y, itr.second[i].z);  //得到 外部探索点对应的 单元ID
        std::cout << "\033[1;32m" << Cell_ID << "\033[0m" << std::endl;
      }
    }

    //    检测   外部已探索点个数  为0   就是没有外部已探索点  融合失败
    for (auto itr : robot_ID_positions_ID_on_keyposegrapher)
    {
      //修改   外部以探索  固定加入起点
      // if(itr.second.empty() || robot_ID_positions_ID_out_keyposegrapher[itr.first].empty())   //若是  外部已探索点集
      // 没有 ,  说明  两个机器人没有共同区域  不融合   或者    外部未探索集没有 说明已探索完毕也不用考虑
      if (robot_ID_positions_ID_out_keyposegrapher[itr.first].empty())
      {
        continue;
        // std::cout << std::endl << "\033[1;32m"<<"不与机器人:" << itr.first<< " 融合"
        // <<robot_ID_positions_ID_on_keyposegrapher[itr.first].size()<< "\033[0m" << std::endl;
      }
      else
      {
        robot_ID_is_fuse[itr.first] = true;  //  连通的且有外部未探索点     就融合成功  这个机器人就得融合进来
      }
    }
    if (robot_ID_is_fuse.empty())
    {
      std::cout << "\033[1;32m"
                << "融合失败，没有满足条件的子图"
                << "\033[0m" << std::endl;
      return false;
    }

    std::cout << "\033[1;32m"
              << "MTSP本地探索点数量： " << exploring_cell_positions.size() << "\033[0m" << std::endl;
    //加入   当前机器人位置点
    int robot_id_ = 0;
    int robot_cell_id_ = exploring_cell_indices.back();
    int robot_position_id_ = exploring_cell_positions.size() - 1;
    geometry_msgs::msg::Point robot_position_ = exploring_cell_positions.back();  //位置数据
    int robot_fuse_grapher_node_id_ = 0;
    Fuse_Grapher_.AddNode(robot_cell_id_, robot_id_, robot_position_id_, robot_fuse_grapher_node_id_, robot_position_);
    Fuse_Grapher_.fuse_nodes_[robot_fuse_grapher_node_id_].is_Explored = true;
    Fuse_Grapher_.fuse_nodes_[robot_fuse_grapher_node_id_].is_robotpose = true;
    //加入 当前起点位置
    int robot_home_id_ = 0;
    int home_position_id_ = -1;
    geometry_msgs::msg::Point home_position_ = keypose_graph->GetFirstKeyposePosition();  //位置数据
    int home_cell_id_ = GetCellInd(home_position_.x, home_position_.y, home_position_.z);
    int home_fuse_grapher_node_id_ = 1;
    Fuse_Grapher_.AddNode(home_cell_id_, robot_home_id_, home_position_id_, home_fuse_grapher_node_id_, home_position_);
    Fuse_Grapher_.fuse_nodes_[home_fuse_grapher_node_id_].is_Explored = true;
    //加入当前起点和当前机器人位置边
    nav_msgs::msg::Path path_tmp_robot_home;
    double distance_robot_home =
        (10 * keypose_graph->GetShortestPath(robot_position_, home_position_, false, path_tmp_robot_home,
                                             false));  //在 关键位姿图上   找到最短距离
    Fuse_Grapher_.AddEdge(robot_fuse_grapher_node_id_, home_fuse_grapher_node_id_, distance_robot_home);
    std::cout << "\033[1;32m"
              << "给本地机器人    " << robot_fuse_grapher_node_id_ << "  和   home  " << home_fuse_grapher_node_id_
              << "    添加边" << distance_robot_home << "\033[0m" << std::endl;

    //添加本地可到达目标点  与   当前点（通过矩阵）、本地可到达目标点（通过矩阵）、外部已探索点（关键位姿图）  之间的边
    for (int i = 0; i < exploring_cell_indices.size() - 1; i++)  // 全局未探索点 不包含起点
    {
      //  添加   local_i  结点
      int local_robot_id_i = 0;
      int local_position_id_i = i;
      geometry_msgs::msg::Point local_position_i = exploring_cell_positions[i];
      int local_cell_id_i = exploring_cell_indices[i];
      int local_fuse_grapher_node_id_i = Fuse_Grapher_.graph_.size();
      bool is_exist_on_i = false;
      fuse_node new_node_on_i(local_cell_id_i, local_robot_id_i, local_position_id_i, local_fuse_grapher_node_id_i,
                              local_position_i);
      is_exist_on_i = Fuse_Grapher_.is_exist_fuse_node(new_node_on_i, local_fuse_grapher_node_id_i);
      if (!is_exist_on_i)  //不存在 添加结点
      {
        // std::cout << "\033[1;32m"<<"不存在外部传入 结点 添加 robot_id: "<< on_robot_id_i<<" 地址:
        // "<<on_position_id_i<<"融合图上编码： "<< on_fuse_grapher_node_id_i<< "\033[0m" << std::endl;
        Fuse_Grapher_.AddNode(local_cell_id_i, local_robot_id_i, local_position_id_i, local_fuse_grapher_node_id_i,
                              local_position_i);
      }
      Fuse_Grapher_.fuse_nodes_[local_fuse_grapher_node_id_i].is_Explored = true;
      //添加  local_i  结点  与    当前点 （通过矩阵） 的边
      double distance_robot_local_i = distance_matrix[i][robot_position_id_];  //直接读取
      bool is_edge_exist_local_i_home = Fuse_Grapher_.is_edge_exist(
          robot_fuse_grapher_node_id_, local_fuse_grapher_node_id_i, distance_robot_local_i);  //存在就直接替换
      if (!is_edge_exist_local_i_home)                                                         //添加边
      {
        std::cout << "\033[1;32m"
                  << "给本地机器人    " << robot_fuse_grapher_node_id_ << "  和   全局未探索点  "
                  << local_fuse_grapher_node_id_i << "    添加边" << distance_robot_local_i << "\033[0m" << std::endl;
        Fuse_Grapher_.AddEdge(robot_fuse_grapher_node_id_, local_fuse_grapher_node_id_i, distance_robot_local_i);
      }
      // 添加 local_i  结点   与    本地可到达目标点（通过矩阵） 的边
      for (int j = 0; j < exploring_cell_indices.size() - 1; j++)
      {
        if (i == j)  // 相同点跳过
        {
          continue;
        }
        //  添加  local_j  结点
        int local_robot_id_j = 0;
        int local_position_id_j = j;
        geometry_msgs::msg::Point local_position_j = exploring_cell_positions[j];
        int local_cell_id_j = exploring_cell_indices[j];
        int local_fuse_grapher_node_id_j = Fuse_Grapher_.graph_.size();
        bool is_exist_on_j = false;
        fuse_node new_node_on_j(local_cell_id_j, local_robot_id_j, local_position_id_j, local_fuse_grapher_node_id_j,
                                local_position_j);
        is_exist_on_j = Fuse_Grapher_.is_exist_fuse_node(new_node_on_j, local_fuse_grapher_node_id_j);
        if (!is_exist_on_j)  //不存在 添加结点
        {
          // std::cout << "\033[1;32m"<<"不存在外部传入 结点 添加 robot_id: "<< on_robot_id_i<<" 地址:
          // "<<on_position_id_i<<"融合图上编码： "<< on_fuse_grapher_node_id_i<< "\033[0m" << std::endl;
          Fuse_Grapher_.AddNode(local_cell_id_j, local_robot_id_j, local_position_id_j, local_fuse_grapher_node_id_j,
                                local_position_j);
        }
        Fuse_Grapher_.fuse_nodes_[local_fuse_grapher_node_id_j].is_Explored = true;
        //  添加 local_i  与 local_j 的 边
        nav_msgs::msg::Path path_tmp;
        double distance_ = distance_matrix[i][j];  //直接读取
        bool is_edge_exist_i_j = Fuse_Grapher_.is_edge_exist(local_fuse_grapher_node_id_i, local_fuse_grapher_node_id_j,
                                                             distance_);  //存在就直接替换
        if (!is_edge_exist_i_j)                                           //添加边
        {
          std::cout << "\033[1;32m"
                    << "给全局未探索点    " << local_fuse_grapher_node_id_i << "  和   全局未探索点 "
                    << local_fuse_grapher_node_id_j << "    添加边" << distance_ << "\033[0m" << std::endl;
          Fuse_Grapher_.AddEdge(local_fuse_grapher_node_id_i, local_fuse_grapher_node_id_j, distance_);
        }
      }

      //添加  local_i  结点   与  外部已探索点（关键位姿图）  之间的边
      for (auto itr : robot_ID_positions_ID_on_keyposegrapher)  //对每个机器人
      {
        if (itr.second.empty())
        {
          continue;
        }
        for (int j = 0; j < itr.second.size(); j++)
        {
          int on_robot_id_j = itr.first;
          int on_position_id_j = itr.second[j];
          geometry_msgs::msg::Point on_position_j = robot_ID_positions_map[itr.first][on_position_id_j];
          int on_cell_id_j = GetCellInd(on_position_j.x, on_position_j.y, on_position_j.z);
          int on_fuse_grapher_node_id_j = Fuse_Grapher_.graph_.size();
          bool is_exist_on_j = false;
          fuse_node new_node_on_j(on_cell_id_j, on_robot_id_j, on_position_id_j, on_fuse_grapher_node_id_j,
                                  on_position_j);
          is_exist_on_j = Fuse_Grapher_.is_exist_fuse_node(new_node_on_j, on_fuse_grapher_node_id_j);
          if (!is_exist_on_j)  //不存在  添加结点
          {
            // std::cout << "\033[1;32m"<<"不存在外部传入  结点 添加 robot_id:  "<< on_robot_id_i<<"  地址:
            // "<<on_position_id_i<<"融合图上编码： "<< on_fuse_grapher_node_id_i<< "\033[0m" << std::endl;
            Fuse_Grapher_.AddNode(on_cell_id_j, on_robot_id_j, on_position_id_j, on_fuse_grapher_node_id_j,
                                  on_position_j);
          }
          Fuse_Grapher_.fuse_nodes_[on_fuse_grapher_node_id_j].is_Explored = true;

          // loca_i  和  外部已探索点j   之间的边   通过  keyposegrapher  计算路径长短
          nav_msgs::msg::Path path_tmp;
          double distance_ = (10 * keypose_graph->GetShortestPath(local_position_i, on_position_j, false, path_tmp,
                                                                  false));  //在 关键位姿图上   找到最短距离
          bool is_edge_exist_i_j = Fuse_Grapher_.is_edge_exist(local_fuse_grapher_node_id_i, on_fuse_grapher_node_id_j,
                                                               distance_);  //存在就直接替换
          if (!is_edge_exist_i_j)                                           //添加边
          {
            std::cout << "\033[1;32m"
                      << "给全局探索点    " << local_fuse_grapher_node_id_i << "  和   外部已探索结点 "
                      << on_fuse_grapher_node_id_j << "    添加边" << distance_ << "\033[0m" << std::endl;
            Fuse_Grapher_.AddEdge(local_fuse_grapher_node_id_i, on_fuse_grapher_node_id_j, distance_);
          }
        }
      }
    }

    std::cout << "\033[1;32m"
              << "把外部已探索结点加入    检测外部探索点与当前机器人位置  ，与 外部未探索点    "
              << "\033[0m" << std::endl;

    //把外部已探索结点加入    检测外部探索点与当前机器人位置  ，与 外部未探索点

    /*****************    屏蔽测试，用起点做      中间点  ******************/

    for (auto itr : robot_ID_positions_ID_on_keyposegrapher)  //对每个机器人
    {
      if (itr.second.empty())  //若是  外部已探索点集   没有 ,  说明  两个机器人没有共同区域  不融合
      {
        continue;
        // std::cout << std::endl << "\033[1;32m"<<"不与机器人:" << itr.first<< " 融合"
        // <<robot_ID_positions_ID_on_keyposegrapher[itr.first].size()<< "\033[0m" << std::endl;
      }
      //对其中一个机器人的   每一个  外部已探索点
      for (int i = 0; i < itr.second.size(); i++)
      {
        int on_robot_id_i = itr.first;
        int on_position_id_i = itr.second[i];
        geometry_msgs::msg::Point on_position_i = robot_ID_positions_map[itr.first][on_position_id_i];
        int on_cell_id_i = GetCellInd(on_position_i.x, on_position_i.y, on_position_i.z);
        int on_fuse_grapher_node_id_i = Fuse_Grapher_.graph_.size();
        bool is_exist_on_i = false;
        fuse_node new_node_on_i(on_cell_id_i, on_robot_id_i, on_position_id_i, on_fuse_grapher_node_id_i,
                                on_position_i);
        is_exist_on_i = Fuse_Grapher_.is_exist_fuse_node(new_node_on_i, on_fuse_grapher_node_id_i);
        if (!is_exist_on_i)  //不存在  添加结点
        {
          // std::cout << "\033[1;32m"<<"不存在外部传入  结点 添加 robot_id:  "<< on_robot_id_i<<"  地址:
          // "<<on_position_id_i<<"融合图上编码： "<< on_fuse_grapher_node_id_i<< "\033[0m" << std::endl;
          Fuse_Grapher_.AddNode(on_cell_id_i, on_robot_id_i, on_position_id_i, on_fuse_grapher_node_id_i,
                                on_position_i);
        }
        Fuse_Grapher_.fuse_nodes_[on_fuse_grapher_node_id_i].is_Explored = true;

        //外部已探索点  和  当前机器人位置   之间的边   通过  keyposegrapher  计算路径长短
        nav_msgs::msg::Path path_tmp;
        double distance_ = (10 * keypose_graph->GetShortestPath(robot_position_, on_position_i, false, path_tmp,
                                                                false));  //在 关键位姿图上   找到最短距离
        bool is_edge_exist = Fuse_Grapher_.is_edge_exist(robot_fuse_grapher_node_id_, on_fuse_grapher_node_id_i,
                                                         distance_);  //存在就直接替换
        if (!is_edge_exist)                                           //添加边
        {
          std::cout << "\033[1;32m"
                    << "给本地机器人    " << robot_fuse_grapher_node_id_ << "  和   外部已探索结点 "
                    << on_fuse_grapher_node_id_i << "    添加边" << distance_ << "\033[0m" << std::endl;
          Fuse_Grapher_.AddEdge(robot_fuse_grapher_node_id_, on_fuse_grapher_node_id_i, distance_);
        }

        //外部已探索点      与  外部已探索点、外部未探索点 之间   读取
        for (int j = 0; j < robot_ID_dis_matrix_map[itr.first][on_position_id_i].size(); j++)
        {
          if (on_position_id_i == j)
          {
            continue;
          }
          int robot_id_j = itr.first;
          int position_id_j = j;
          geometry_msgs::msg::Point position_j = robot_ID_positions_map[itr.first][position_id_j];
          int cell_id_j = GetCellInd(position_j.x, position_j.y, position_j.z);
          int fuse_grapher_node_id_j = Fuse_Grapher_.graph_.size();
          bool is_exist_j = false;
          fuse_node new_node_j(cell_id_j, robot_id_j, position_id_j, fuse_grapher_node_id_j, position_j);
          is_exist_j = Fuse_Grapher_.is_exist_fuse_node(new_node_j, fuse_grapher_node_id_j);
          if (!is_exist_j)  //不存在  添加结点
          {
            // std::cout << "\033[1;32m"<<"不存在外部传入  结点 添加 robot_id:  "<< robot_id_j<<"  地址:
            // "<<position_id_j<<"融合图上编码： "<< fuse_grapher_node_id_j<< "\033[0m" << std::endl;
            Fuse_Grapher_.AddNode(cell_id_j, robot_id_j, position_id_j, fuse_grapher_node_id_j, position_j);
          }
          //检测边   i,j
          //检测  i,j  的边   这个边是从矩阵上得到
          double distance_ij = robot_ID_dis_matrix_map[itr.first][on_position_id_i][j];  //两个结点的距离信息
          bool is_edge_exist = Fuse_Grapher_.is_edge_exist(on_fuse_grapher_node_id_i, fuse_grapher_node_id_j,
                                                           distance_ij);  //存在就直接替换
          if (!is_edge_exist)                                             //添加边
          {
            std::cout << "\033[1;32m"
                      << "给外部已探索结点    " << on_fuse_grapher_node_id_i
                      << "  和     外部结点（可能是已探索  可能是未探索）" << fuse_grapher_node_id_j << " 添加边"
                      << distance_ij << "\033[0m" << std::endl;
            Fuse_Grapher_.AddEdge(on_fuse_grapher_node_id_i, fuse_grapher_node_id_j, distance_ij);
          }
        }
      }
    }

    //第三步     本地探索结点+外部探索结点      结点都存在了   添加外部   结点互相之间的边。
    for (auto itr : robot_ID_positions_ID_out_keyposegrapher)
    {
      if (robot_ID_is_fuse[itr.first] == false)  //融合失败就跳过
      {
        continue;
      }
      for (int i = 0; i < itr.second.size(); i++)
      {
        //在  融合图上查找  找到       机器人名为  irt.first,   位置编号为 irt.second[i]的 结点     在融合图上的编号
        int out_robot_id_i = itr.first;
        int out_i = itr.second[i];
        int out_fuse_grapher_node_id_i;
        bool is_exist_i = Fuse_Grapher_.FindNode_id(out_robot_id_i, out_i, out_fuse_grapher_node_id_i);
        if (!is_exist_i)  //不存在跳出
        {
          //添加结点
          out_fuse_grapher_node_id_i = Fuse_Grapher_.graph_.size();
          geometry_msgs::msg::Point out_position_i = robot_ID_positions_map[itr.first][out_i];
          int out_cell_id_i = GetCellInd(out_position_i.x, out_position_i.y, out_position_i.z);
          Fuse_Grapher_.AddNode(out_cell_id_i, out_robot_id_i, out_i, out_fuse_grapher_node_id_i, out_position_i);
        }
        for (int j = 0; j < itr.second.size(); j++)
        {
          if (i == j)
          {
            continue;
          }
          int out_robot_id_j = itr.first;
          int out_j = itr.second[j];
          int out_fuse_grapher_node_id_j;
          bool is_exist_j = Fuse_Grapher_.FindNode_id(out_robot_id_j, out_j, out_fuse_grapher_node_id_j);
          if (!is_exist_j)  //不存在跳出
          {
            //添加结点
            out_fuse_grapher_node_id_j = Fuse_Grapher_.graph_.size();
            geometry_msgs::msg::Point out_position_j = robot_ID_positions_map[itr.first][out_j];
            int out_cell_id_j = GetCellInd(out_position_j.x, out_position_j.y, out_position_j.z);
            Fuse_Grapher_.AddNode(out_cell_id_j, out_robot_id_j, out_j, out_fuse_grapher_node_id_j, out_position_j);
          }
          //添加边   外部未探索点 i   和外部未探索点 j 之间的边    边长度是读取
          double distance_ij = robot_ID_dis_matrix_map[itr.first][out_i][out_j];
          bool is_edge_exist = Fuse_Grapher_.is_edge_exist(out_fuse_grapher_node_id_i, out_fuse_grapher_node_id_j,
                                                           distance_ij);  //存在就直接替换
          if (!is_edge_exist)                                             //添加边
          {
            std::cout << "\033[1;32m"
                      << "外部未探索结点    " << out_fuse_grapher_node_id_i << "  和   外部未探索结点  "
                      << out_fuse_grapher_node_id_j << " 添加边" << distance_ij << "\033[0m" << std::endl;
            Fuse_Grapher_.AddEdge(out_fuse_grapher_node_id_i, out_fuse_grapher_node_id_j, distance_ij);
          }
        }

        // 给外部未探索结点  和  home  添加边，这个边是用于以防万一没有  外部已探索结点时  寻求路径的
        double distance_i_home = 9999;  //输出是 int  不能超过  范围
        bool is_edge_exist = Fuse_Grapher_.is_edge_exist(out_fuse_grapher_node_id_i, home_fuse_grapher_node_id_,
                                                         distance_i_home);  //存在就直接替换
        if (!is_edge_exist)                                                 //添加边
        {
          std::cout << "\033[1;32m"
                    << "home    " << home_fuse_grapher_node_id_ << "  和   外部未探索结点  "
                    << out_fuse_grapher_node_id_i << " 添加边 " << distance_i_home << "\033[0m" << std::endl;
          Fuse_Grapher_.AddEdge(out_fuse_grapher_node_id_i, home_fuse_grapher_node_id_, distance_i_home);
        }
      }
    }

    geometry_msgs::msg::Point robot_position = exploring_cell_positions.back();
    int robot_cell_id = GetCellInd(robot_position.x, robot_position.y, robot_position.z);
    int robot_fuse_grapher_node_id;

    std::map<int, int> robot_ID_robotposetionfusegrapher_id;  //机器人位置    在融合图上的位置信息
    //第5步      检测外部机器人位置
    for (auto itr : robot_ID_positions_map)
    {
      if (robot_ID_is_fuse[itr.first] == false)  //融合失败就跳过
      {
        continue;
      }
      int out_robot_id = itr.first;
      int out_robot_position_id = itr.second.size() - 1;
      int out_robot_fuse_grapher_node_id;
      //查询  融合途中是否有
      bool is_exist_out_robot =
          Fuse_Grapher_.FindNode_id(out_robot_id, out_robot_position_id, out_robot_fuse_grapher_node_id);
      if (is_exist_out_robot)
      {
        //存在就记录当前的机器人在融合图上的编号
        robot_ID_robotposetionfusegrapher_id.insert(std::pair<int, int>(itr.first, out_robot_fuse_grapher_node_id));
        Fuse_Grapher_.fuse_nodes_[out_robot_fuse_grapher_node_id].is_robotpose = true;
      }
    }

    //第六步  挑选   外部探索点  及 本地机器人位置   做   MTSP的数据输出
    int robot_ip;
    if (!robot_ID_robotposetionfusegrapher_id.empty())  //若不为空     则融合成功
    {
      //选中   外部探索点       及      本地机器人位置    作为     MTSP的  输入
      std::vector<geometry_msgs::msg::Point> exploring_cell_positions_fuse_grapher;  //融合图   结点位置  数据集
      std::vector<int> exploring_cell_indices_fuse_grapher;  //融合图  结点  所在 Cell  单元  编码
      std::vector<int> exploring_fuse_grapher_node_id;  //融合图  结点  所在   图上的编号   为了求距离
      std::vector<int> robot_start_id_on_exploring_cell_positions_fuse_grapher;  //  多机器人  起点
      std::vector<int> robot_start_id_on_MTSP;                                   //  多机器人  起点   在
      int fuse_grapher_nodes_num = Fuse_Grapher_.fuse_nodes_.size();             //融合图  结点数
      for (int i = 0; i < fuse_grapher_nodes_num; i++)                           //对所有融合节点
      {
        int robot_id = Fuse_Grapher_.fuse_nodes_[i].robot_id_;
        int cell_id = Fuse_Grapher_.fuse_nodes_[i].Cell_id_;
        if (Fuse_Grapher_.fuse_nodes_[i].robot_id_ != 0)  //不是本地的结点   且  在外部探索点上  要加入MTSP  的里面
        {
          bool is_robot = false;
          for (auto itr : robot_ID_robotposetionfusegrapher_id)  //加入机器人位置
          {
            if (Fuse_Grapher_.fuse_nodes_[i].robot_id_ == itr.first &&
                Fuse_Grapher_.fuse_nodes_[i].positions_id_ ==
                    robot_ID_positions_map[itr.first].size() - 1)  // 机器人位置
            {
              is_robot = true;
              exploring_cell_positions_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].position_);
              exploring_fuse_grapher_node_id.push_back(Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_);
              std::cout << "机器人：" << itr.first << "   在融合图上的编号： "
                        << Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_ << std::endl;
              exploring_cell_indices_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].Cell_id_);
              robot_start_id_on_MTSP.push_back(exploring_cell_positions_fuse_grapher.size() - 1);
            }
          }

          //  加入探索点
          std::vector<int>::iterator ret = std::find(robot_ID_positions_ID_out_keyposegrapher[robot_id].begin(),
                                                     robot_ID_positions_ID_out_keyposegrapher[robot_id].end(),
                                                     Fuse_Grapher_.fuse_nodes_[i].positions_id_);
          if (ret == robot_ID_positions_ID_out_keyposegrapher[robot_id].end())  //指向最后，则没有找到
          {
            continue;  //不是外部未探索点  则跳过这个结点
          }
          //是已探索点  就不选。
          if (subspaces_world_->GetCell(cell_id).GetStatus() == CellStatus::COVERED ||
              subspaces_world_->GetCell(cell_id).GetStatus() == CellStatus::COVERED_BY_OTHERS)
            continue;
          if (!is_robot)  //普通点  外部未探索点
          {
            exploring_cell_positions_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].position_);
            exploring_fuse_grapher_node_id.push_back(Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_);
            std::cout << "机器人：" << Fuse_Grapher_.fuse_nodes_[i].robot_id_
                      << "   选中外部未探索结点  在融合图上的编号 ： "
                      << Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_ << std::endl;
            exploring_cell_indices_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].Cell_id_);
          }
        }
        else if (Fuse_Grapher_.fuse_nodes_[i].robot_id_ == 0 &&
                 Fuse_Grapher_.fuse_nodes_[i].positions_id_ == exploring_cell_positions.size() - 1)  //当前机器人点
        {
          exploring_cell_positions_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].position_);
          exploring_fuse_grapher_node_id.push_back(Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_);
          std::cout << "本地机器人    在融合图上的编号： " << Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_
                    << std::endl;
          exploring_cell_indices_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].Cell_id_);
          robot_ip = exploring_cell_positions_fuse_grapher.size() - 1;
        }
        else if (Fuse_Grapher_.fuse_nodes_[i].robot_id_ == 0 &&
                 Fuse_Grapher_.fuse_nodes_[i].positions_id_ != exploring_cell_positions.size() - 1 &&
                 Fuse_Grapher_.fuse_nodes_[i].positions_id_ != -1)  //添加全局可到达探索点
        {
          //加入   全局可到达探索结点
          exploring_cell_positions_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].position_);
          exploring_fuse_grapher_node_id.push_back(Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_);
          std::cout << "机器人：" << Fuse_Grapher_.fuse_nodes_[i].robot_id_
                    << "   选中全局可到达探索结点  在融合图上的编号 ： "
                    << Fuse_Grapher_.fuse_nodes_[i].fuse_grapher_node_id_ << std::endl;
          exploring_cell_indices_fuse_grapher.push_back(Fuse_Grapher_.fuse_nodes_[i].Cell_id_);
        }
      }

      int MTSP_node_num = exploring_cell_positions_fuse_grapher.size();
      std::vector<std::vector<int>> distance_matrix_fuse_grapher(
          MTSP_node_num, std::vector<int>(MTSP_node_num, 0));  //融合图的距离矩阵
      //构建距离矩阵
      for (int i = 0; i < MTSP_node_num; i++)
      {
        for (int j = 0; j < i; j++)
        {
          if (!use_keypose_graph_ || keypose_graph == nullptr ||
              keypose_graph->GetNodeNum() == 0)  //不使用关键位姿图   直接用两点 直线距离构造边
          {
            // Use straight line connection     初始化  使用直线连接
            distance_matrix_fuse_grapher[i][j] = static_cast<int>(
                10 * misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
                         exploring_cell_positions_fuse_grapher[i], exploring_cell_positions_fuse_grapher[j]));
          }
          else
          {
            // Use keypose graph   使用关键位姿图
            nav_msgs::msg::Path path_tmp;
            distance_matrix_fuse_grapher[i][j] = static_cast<int>(
                10 * GetShortestPath_Fuse_Grapher(Fuse_Grapher_, exploring_fuse_grapher_node_id[i],
                                                  exploring_fuse_grapher_node_id[j]));  //在 关键位姿图上   找到最短距离
            std::cout << "融合图上   " << exploring_fuse_grapher_node_id[i] << "     和   "
                      << exploring_fuse_grapher_node_id[j] << "    路径长度  " << distance_matrix_fuse_grapher[i][j]
                      << std::endl;
          }
        }
      }
      //对角化
      for (int i = 0; i < MTSP_node_num; i++)
      {
        for (int j = i + 1; j < MTSP_node_num; j++)
        {
          distance_matrix_fuse_grapher[i][j] = distance_matrix_fuse_grapher[j][i];
        }
      }
      std::cout << std::endl
                << "\033[1;32m"
                << "成功"
                << "\033[0m" << std::endl;
      //赋值   传出
      exploring_cell_indices_MTSP = exploring_cell_indices_fuse_grapher;
      std::cout << "\033[1;32m"
                << "融合图结点单元 Cell _ID: "
                << "\033[0m" << std::endl;
      for (int j = 0; j < exploring_cell_indices_fuse_grapher.size(); j++)
      {
        std::cout << "\033[1;32m" << exploring_cell_indices_fuse_grapher[j] << "\033[0m" << std::endl;
      }
      exploring_cell_positions_MTSP = exploring_cell_positions_fuse_grapher;
      distance_matrix_MTSP = distance_matrix_fuse_grapher;
      robot_position_id_on_exploring_cell_positions_MTSP.push_back(robot_ip);  //   本地机器人放第一个
      positions_fuse_grapher_node_id = exploring_fuse_grapher_node_id;
      for (int i = 0; i < robot_start_id_on_MTSP.size(); i++)
      {
        robot_position_id_on_exploring_cell_positions_MTSP.push_back(robot_start_id_on_MTSP[i]);
      }
      return true;
    }
    else
    {  //融合失败   没用共同的点
      std::cout << std::endl
                << "\033[1;32m"
                << " 融合失败  没有共同点"
                << "\033[0m" << std::endl;
      return false;
    }
  }
  else  //没有外部输入子图
  {
    std::cout << std::endl
              << "\033[1;32m"
              << "   融合失败  没有传入子图"
              << "\033[0m" << std::endl;
    return false;  //融合失败    做TSP
  }
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

//添加函数    将路径添加到  keyposegrapher    中  返回true  添加成功  false   添加失败
bool GridWorld::UpDateIntermediate2GoalPath(const nav_msgs::msg::Path& this_path,
                                            std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  //更新中间节点 并检查是否          目标点 是否是       误点
  if (this_path.poses.size() <= 2)
    return false;
  //将路径加入到  keyposegrapher   图上 ，直接计算当前位置到目标位置的路径
  //当 目标位置在局部规划框内，可能会被  障碍物将这个   keyposegrapher  的节点排除、
  //从而  排查出误点
  bool is_connect = false;
  keypose_graph->AddPath(this_path);  //直接添加 路径   不论连接成功与否，加了没坏处
  //更新   网格  里面的   连通  点   数      subspaces_->GetCell(cell_index).GetGraphNodeIndices()
  for (int i = 0; i < this_path.poses.size(); i++)
  {
    exploration_path_ns::Node node_temp;
    geometry_msgs::msg::Point point_temp;
    point_temp = this_path.poses[i].pose.position;
    int KeyposeID = keypose_graph->GetClosestNodeInd(point_temp);
    int cell_index = GetCellInd(point_temp.x, point_temp.y, point_temp.z);
    if (subspaces_->InRange(cell_index))  //单元  在子空间 范围内      子空间范围
    {
      subspaces_->GetCell(cell_index).AddGraphNode(KeyposeID);  //   将子空间范围内  新增连接单元  状态为探索的单元
                                                                //   新增图结点
    }
  }
  bool is_ok = false;
  for (int i = 0; i < this_path.poses.size(); i++)
  {
    geometry_msgs::msg::Point point_temp;
    point_temp = this_path.poses[i].pose.position;
    //查询  中间节点  是否能到达
    //中间节点  在 本地网格状态里面是   覆盖
    // bool is_reachable=keypose_graph->IsPositionReachable(point_temp,kCellSize/2);
    //  加点并连边
    int cell_index_ = GetCellInd(point_temp.x, point_temp.y, point_temp.z);
    // CellStatus cell_status_local=GetCellStatus(cell_index);  //本地
    // CellStatus cell_status_world=GetCellStatus_world(cell_index);//全局
    //当前结点是  在本地状态  中  是覆盖  和  探索   即可加入到    keyposegrapher  中  查找最近的结点连接

    //加点并添加边
    double min_dist = DBL_MAX;
    double min_dist_node_ind = -1;
    //查询点在keyposegrapher 图上的  id
    int this_node_index = keypose_graph->GetClosestNodeInd(point_temp);

    for (const auto& node_ind : subspaces_->GetCell(cell_index_).GetGraphNodeIndices())  //相同  单元      存在图结点
    {
      geometry_msgs::msg::Point node_position = keypose_graph->GetNodePosition(node_ind);  //单元内  存在于图上的结点集
      double dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(node_position, point_temp);
      //不需要条件，只要在同样的单元格内连接即可。
      keypose_graph->AddEdge(node_ind, this_node_index, dist);
      std::cout << "添加边" << std::endl;
      is_connect = true;
      if (dist < min_dist)  //找到当前单元图结点上      距离  外部给入探索点  最近的距离
      {
        min_dist = dist;
        min_dist_node_ind = node_ind;
      }
    }
  }

  //另一种思路   求当前点距离小于   kCellSize    的所有点   连接即可。
  keypose_graph->UpdateNodes();
  return is_connect;
}

//新增 发布  网格子图
void GridWorld::GetSubgraph(tare_planner::msg::Subgraph& pub_subgraph)
{
  //搜集 邻接子网格中的数据    neighbor_cell_indices_
  for (int i = 0; i < neighbor_cell_indices_.size(); i++)
  {
    int cell_index = neighbor_cell_indices_[i];
    std::vector<int> temp_vec;
    tare_planner::msg::Subnode temp_node;
    temp_node.cell_index = cell_index;
    // GetCellInd(cell_index)
    temp_node.cell_statu = subspaces_world_->GetCell(cell_index).GetStatusInt();
    temp_node.is_roadmap_set = subspaces_->GetCell(cell_index).IsRoadmapConnectionPointSet();
    temp_node.postion.x = subspaces_->GetCell(cell_index).GetRoadmapConnectionPoint()[0];
    temp_node.postion.y = subspaces_->GetCell(cell_index).GetRoadmapConnectionPoint()[1];
    temp_node.postion.z = subspaces_->GetCell(cell_index).GetRoadmapConnectionPoint()[2];
    temp_node.robot_id = pub_subgraph.robot_id;
    temp_node.connect_cell_index = temp_vec;
    pub_subgraph.neighbor_nodes.push_back(temp_node);
  }
}

//新增  更新网格子图
void GridWorld::UpdateMTSPgridgraph(tare_planner::msg::Subgraph& cur_subgraph,
                                    std::map<int, std::map<int, tare_planner::msg::Subnode>>& robot_cell_subnode_,
                                    std::map<int, std::vector<geometry_msgs::msg::Point>>& other_robot_history_position_,
                                    int robot_num, double kAddNodeMinDist, double kAddEdgeConnectDistThr,
                                    const std::unique_ptr<planning_env_ns::PlanningEnv>& planning_env_,
                                    double kAddEdgeCollisionCheckResolution,
                                    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager_)
{
  int count_add_edge = 0;
  int count_add_node = 0;
  // std::cout<<"添加  外部机器人  子网格结点 "<<std::endl;
  for (auto& itr_robot : robot_cell_subnode_)  //循环机器人ID
  {
    if (MTSP_grid_graph_.find_map_.count(itr_robot.first) == 0)  //不存在就添加（或者在初始化先行定义？减少每次查询？）
    {
      std::unordered_map<int, int> temp_map;
      MTSP_grid_graph_.find_map_.insert(std::pair<int, std::unordered_map<int, int>>(itr_robot.first, temp_map));
    }
    // std::cout<<"添加  外部机器人"<< itr_robot.first <<"  子网格结点  个数  ： "<<itr_robot.second.size()<<std::endl;
    for (auto& itr_cell : itr_robot.second)
    {
      //查询在      std::unordered_map<int,std::unordered_map<int,int>> find_map_    是否已经存在这个点了
      if (MTSP_grid_graph_.find_map_[itr_robot.first].count(itr_cell.first) == 0)  //不存在  需要添加结点
      {
        if (itr_cell.second.connect_cell_index.size() <= 0)
        {
          continue;
        }
        //添加结点到  find_map_    编号为 未加入当前点的数量（从0开始的编号地址）
        MTSP_grid_graph_.find_map_[itr_robot.first].insert(
            std::pair<int, int>(itr_cell.first, MTSP_grid_graph_.MTSP_graph_nodes_.size()));

        //添加结点到  MTSP_graph_nodes_   及  边容器
        MTSP_grid_graph_.AddNode(itr_robot.first, itr_cell.first, itr_cell.second.postion, false,
                                 MTSP_grid_graph_.MTSP_graph_nodes_.size());  //不是机器人位置 keypose
      }
      else
      {
        //存在       更新  点   的    位置信息  毕竟  是可能改变的         roadmap——position
        int new_cell_index =
            GetCellInd(itr_cell.second.postion.x, itr_cell.second.postion.y, itr_cell.second.postion.z);
        int node_index_MTSP = MTSP_grid_graph_.find_map_[itr_robot.first][itr_cell.first];
        int num_old = MTSP_grid_graph_.graph_[node_index_MTSP].size();
        int num_new = itr_cell.second.connect_cell_index.size();
        if (new_cell_index == MTSP_grid_graph_.MTSP_graph_nodes_[node_index_MTSP].cell_id_ &&
            !MTSP_grid_graph_.MTSP_graph_nodes_[node_index_MTSP].is_connected_ && num_old < num_new)
        {
          MTSP_grid_graph_.MTSP_graph_nodes_[node_index_MTSP].position_.x = itr_cell.second.postion.x;
          MTSP_grid_graph_.MTSP_graph_nodes_[node_index_MTSP].position_.y = itr_cell.second.postion.y;
          MTSP_grid_graph_.MTSP_graph_nodes_[node_index_MTSP].position_.z = itr_cell.second.postion.z;
        }
      }
    }
  }
  if (MTSP_grid_graph_.find_map_.count(cur_subgraph.robot_id) ==
      0)  //不存在就添加（或者在初始化先行定义？减少每次查询？）
  {
    std::unordered_map<int, int> temp_map;
    MTSP_grid_graph_.find_map_.insert(std::pair<int, std::unordered_map<int, int>>(cur_subgraph.robot_id, temp_map));
  }
  // std::cout<<"添加  本地机器人  子网格结点  个数  ： "<<cur_subgraph.neighbor_nodes.size()<<std::endl;
  for (int i = 0; i < cur_subgraph.neighbor_nodes.size(); i++)
  {
    if (cur_subgraph.neighbor_nodes[i].connect_cell_index.size() <= 0)
    {
      continue;
    }
    if (MTSP_grid_graph_.find_map_[cur_subgraph.robot_id].count(cur_subgraph.neighbor_nodes[i].cell_index) ==
        0)  //不存在  需要添加结点
    {
      count_add_node++;
      MTSP_grid_graph_.find_map_[cur_subgraph.robot_id].insert(
          std::pair<int, int>(cur_subgraph.neighbor_nodes[i].cell_index, MTSP_grid_graph_.MTSP_graph_nodes_.size()));

      //添加结点到  MTSP_graph_nodes_   及  边容器
      MTSP_grid_graph_.AddNode(cur_subgraph.robot_id, cur_subgraph.neighbor_nodes[i].cell_index,
                               cur_subgraph.neighbor_nodes[i].postion, false,
                               MTSP_grid_graph_.MTSP_graph_nodes_.size());  //不是机器人位置 keypose
    }
    else
    {
      int new_cell_index =
          GetCellInd(cur_subgraph.neighbor_nodes[i].postion.x, cur_subgraph.neighbor_nodes[i].postion.y,
                     cur_subgraph.neighbor_nodes[i].postion.z);
      int node_index_MTSP =
          MTSP_grid_graph_.find_map_[cur_subgraph.robot_id][cur_subgraph.neighbor_nodes[i].cell_index];
      if (MTSP_grid_graph_.MTSP_graph_nodes_[node_index_MTSP].cell_id_ == new_cell_index)
      {
        MTSP_grid_graph_.MTSP_graph_nodes_[node_index_MTSP].position_.x = cur_subgraph.neighbor_nodes[i].postion.x;
        MTSP_grid_graph_.MTSP_graph_nodes_[node_index_MTSP].position_.y = cur_subgraph.neighbor_nodes[i].postion.y;
        MTSP_grid_graph_.MTSP_graph_nodes_[node_index_MTSP].position_.z = cur_subgraph.neighbor_nodes[i].postion.z;
      }
    }
  }
  for (auto& itr_robot : robot_cell_subnode_)  //对每个机器人
  {
    for (auto& itr_cell : itr_robot.second)  //对每个机器人的每个邻接结点
    {
      if (MTSP_grid_graph_.find_map_[itr_robot.first].count(itr_cell.first) == 0)
      {
        continue;
      }
      int from_node_ind = MTSP_grid_graph_.find_map_[itr_robot.first][itr_cell.first];
      //查询之前的边连接信息  ，对于   同机器人cell_index  的边    进行维护   裁剪或增加
      for (int i = 0; i < MTSP_grid_graph_.graph_[from_node_ind].size(); i++)
      {
        int to_node_ind = MTSP_grid_graph_.graph_[from_node_ind][i];
        if (MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].robot_id_ == itr_robot.first &&
            MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].is_keypose_ == false)  //来自同一个机器人的cell
                                                                                   //且不是历史位置的  连接结点
        {
          //查询  当前的  connect_cell_index是否存在这个结点，  存在就更新边大小  ，不存在就删除当前边
          if (std::find(itr_cell.second.connect_cell_index.begin(), itr_cell.second.connect_cell_index.end(),
                        MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].cell_id_) ==
              itr_cell.second.connect_cell_index.end())  //等于end()  不存在，存在就返回索引
          {
            // //删除边
            MTSP_grid_graph_.DelEdge(from_node_ind, to_node_ind);
          }
          else
          {
            //存在就   更新边大小
            double distance = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
                MTSP_grid_graph_.MTSP_graph_nodes_[from_node_ind].position_,
                MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].position_);
            // std::cout<<"更新外部边   "<<from_node_ind<<"  to "<<to_node_ind <<std::endl;
            if (distance < 2 * kCellSize)
            {
              MTSP_grid_graph_.UpdateEdge(from_node_ind, to_node_ind, distance);
            }
            else
            {
              MTSP_grid_graph_.DelEdge(from_node_ind, to_node_ind);
            }
          }
        }
      }

      //对本次传入的进行增添
      for (int i = 0; i < itr_cell.second.connect_cell_index.size(); i++)  //对于每个机器人的每个邻接结点的 所有连接边
      {
        //保险   查询当前是否存在对应的结点
        if (MTSP_grid_graph_.find_map_[itr_robot.first].count(itr_cell.second.connect_cell_index[i]))
        {
          int to_node_ind = MTSP_grid_graph_.find_map_[itr_robot.first][itr_cell.second.connect_cell_index[i]];
          if (!MTSP_grid_graph_.is_edge_exist(from_node_ind, to_node_ind))  //不存在  边   两个点都存在
          {
            //添加边
            double distance = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
                MTSP_grid_graph_.MTSP_graph_nodes_[from_node_ind].position_,
                MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].position_);
            // std::cout<<"添加外部边   "<<from_node_ind<<"  to "<<to_node_ind <<std::endl;
            if (distance < 2 * kCellSize)
            {
              MTSP_grid_graph_.AddEdge(from_node_ind, to_node_ind, distance);
            }
          }
        }
      }
      if (!(itr_cell.second.cell_statu == 2 || itr_cell.second.cell_statu == 3))  //不满足   状态为已探索就跳过
      {
        continue;
      }
      //状态为已探索  才进行额外的加边
      for (int i = 1; i <= robot_num; i++)  // 1,2,3
      {
        if (i == itr_robot.first)  //相同跳过
        {
          continue;
        }
        if (MTSP_grid_graph_.find_map_[i].count(itr_cell.first))
        {
          int to_node_ind = MTSP_grid_graph_.find_map_[i][itr_cell.first];  //同一网格里的点
          //写一个函数判定下诉    添加额外边的条件          传入两个索引 + 距离参数+planning_env  函数
          // std::cout<<"添加额外边   "<<from_node_ind<<"  to "<<to_node_ind <<std::endl;
          IsAddExtraEdge(from_node_ind, to_node_ind, kAddNodeMinDist, planning_env_, kAddEdgeCollisionCheckResolution);
        }
      }
    }
  }
  for (int k = 0; k < cur_subgraph.neighbor_nodes.size(); k++)
  {
    if (MTSP_grid_graph_.find_map_[cur_subgraph.robot_id].count(cur_subgraph.neighbor_nodes[k].cell_index) == 0)
    {
      continue;
    }

    int from_node_ind = MTSP_grid_graph_.find_map_[cur_subgraph.robot_id][cur_subgraph.neighbor_nodes[k].cell_index];

    //查询之前的边连接信息  ，对于   同机器人cell_index  的边    进行维护   裁剪或增加

    //测试不删除   边效果会不会好
    for (int i = 0; i < MTSP_grid_graph_.graph_[from_node_ind].size(); i++)
    {
      int to_node_ind = MTSP_grid_graph_.graph_[from_node_ind][i];
      if (MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].robot_id_ == cur_subgraph.robot_id &&
          MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].is_keypose_ == false)  //来自同一个机器人的cell
                                                                                 //且不是历史位置的  连接结点
      {
        //查询  当前的  connect_cell_index  是否存在这个连接点，  存在就更新边大小  ，不存在就删除当前边
        if (std::find(cur_subgraph.neighbor_nodes[k].connect_cell_index.begin(),
                      cur_subgraph.neighbor_nodes[k].connect_cell_index.end(),
                      MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].cell_id_) ==
            cur_subgraph.neighbor_nodes[k].connect_cell_index.end())  //等于end()  不存在，存在就返回索引
        {
          //删除边
          //  测试    不删除以前的边，只更新

          // std::cout<<"删除边   "<<from_node_ind<<"  to "<<to_node_ind <<std::endl;
          MTSP_grid_graph_.DelEdge(from_node_ind, to_node_ind);
        }
        else
        {
          //存在就   更新边大小
          double distance = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
              MTSP_grid_graph_.MTSP_graph_nodes_[from_node_ind].position_,
              MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].position_);
          // std::cout<<"更新边   "<<from_node_ind<<"  to "<<to_node_ind <<std::endl;
          MTSP_grid_graph_.UpdateEdge(from_node_ind, to_node_ind, distance);
        }
      }
    }

    //对本次传入的进行增添
    for (int j = 0; j < cur_subgraph.neighbor_nodes[k].connect_cell_index.size(); j++)
    {
      //保险   查询当前是否存在对应的结点
      if (MTSP_grid_graph_.find_map_[cur_subgraph.robot_id].count(
              cur_subgraph.neighbor_nodes[k].connect_cell_index[j]) != 0)  //存在
      {
        int to_node_ind =
            MTSP_grid_graph_.find_map_[cur_subgraph.robot_id][cur_subgraph.neighbor_nodes[k].connect_cell_index[j]];
        if (!MTSP_grid_graph_.is_edge_exist(from_node_ind, to_node_ind))  //不存在  边   两个点都存在
                                                                          //边存在的情况在上一个已经更新了
        {
          //添加边
          double distance = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
              MTSP_grid_graph_.MTSP_graph_nodes_[from_node_ind].position_,
              MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].position_);
          count_add_edge++;
          if (distance < 2 * kCellSize)
          {
            MTSP_grid_graph_.AddEdge(from_node_ind, to_node_ind, distance);
          }
        }
      }
    }
    if (!(cur_subgraph.neighbor_nodes[k].cell_statu == 2 ||
          cur_subgraph.neighbor_nodes[k].cell_statu == 3))  //不满足  状态是已探索   就跳过
    {
      continue;
    }
    for (int i = 1; i <= robot_num; i++)  // 1,2,3
    {
      if (i == cur_subgraph.robot_id)  //相同跳过
      {
        continue;
      }
      if (MTSP_grid_graph_.find_map_[i].count(cur_subgraph.neighbor_nodes[k].cell_index))  //外部传入了这个点   添加边
      {
        int to_node_ind = MTSP_grid_graph_.find_map_[i][cur_subgraph.neighbor_nodes[k].cell_index];
        // std::cout<<"尝试添加额外边   "<<from_node_ind<<"  to "<<to_node_ind <<std::endl;
        IsAddExtraEdge(from_node_ind, to_node_ind, kAddNodeMinDist, planning_env_, kAddEdgeCollisionCheckResolution);
      }
    }
  }
  for (auto& itr_robot : other_robot_history_position_)
  {
    if (MTSP_grid_graph_.last_robot_position_.count(itr_robot.first) == 0)  //不存在  就初始化
    {
      geometry_msgs::msg::Point temp;
      temp.x = 0;
      temp.y = 0;
      temp.z = 0;
      MTSP_grid_graph_.last_robot_position_.insert(std::pair<int, geometry_msgs::msg::Point>(itr_robot.first, temp));
      int temp_node_index = -1;
      MTSP_grid_graph_.last_robot_node_index_.insert(std::pair<int, int>(itr_robot.first, temp_node_index));
    }

    for (int i = 0; i < itr_robot.second.size(); i++)
    {
      //是否  添加点  （添加点就必然添加边） 到  MTSP_grid_graph
      double distance = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
          MTSP_grid_graph_.last_robot_position_[itr_robot.first], itr_robot.second[i]);
      if (distance < kAddNodeMinDist)  //距离小于最小添加距离  就不加入
      {
        continue;
      }
      //添加点
      //  查询是否在 范围内存在点

      int robot_index_MTSP_grid_graph;
      double dist;
      //当前点和连通树   在   距离小于    kAddNodeMinDist  中存在点  就不添加点而直接使用查找到的点
      bool is_range_exist = MTSP_grid_graph_.GetCloseConnectNodeMtspGraph(
          itr_robot.second[i], robot_index_MTSP_grid_graph, dist, kAddNodeMinDist);
      if (!is_range_exist)  //不存在添加点  存在就不添加点
      {
        robot_index_MTSP_grid_graph = MTSP_grid_graph_.MTSP_graph_nodes_.size();
        int robot_cell_id = GetCellInd(itr_robot.second[i].x, itr_robot.second[i].y, itr_robot.second[i].z);
        MTSP_grid_graph_.AddNode(itr_robot.first, robot_cell_id, itr_robot.second[i], true,
                                 MTSP_grid_graph_.MTSP_graph_nodes_.size());
      }

      //检测  当前传入网格 被置为  探索状态的机器人位置
      for (auto& itr_ : robot_cell_subnode_[itr_robot.first])
      {
        int cell_index = itr_.first;
        if (itr_.second.cell_statu == 1 &&
            subspaces_world_->GetCell(cell_index).GetMTSP_graph_index_find_exploring_cell_() == -1)
        {
          subspaces_world_->GetCell(cell_index).SetMTSP_graph_index_find_exploring_cell_(robot_index_MTSP_grid_graph);
        }
      }

      //是否添加边
      if (MTSP_grid_graph_.last_robot_node_index_[itr_robot.first] != -1)  //上一个为-1  则是初始化  跳过，只添加点即可
      {
        //添加与前一个路经点的 边
        //判定是否存在边
        if (!MTSP_grid_graph_.is_edge_exist(MTSP_grid_graph_.last_robot_node_index_[itr_robot.first],
                                            robot_index_MTSP_grid_graph))
        {
          MTSP_grid_graph_.AddEdge(MTSP_grid_graph_.last_robot_node_index_[itr_robot.first],
                                   robot_index_MTSP_grid_graph, distance);
        }
      }
      ConnectRobotToNeighborCellGraphNode(robot_index_MTSP_grid_graph, kCellSize / 2, planning_env_,
                                          kAddEdgeCollisionCheckResolution);
      //更新  最后点
      MTSP_grid_graph_.last_robot_node_index_[itr_robot.first] = robot_index_MTSP_grid_graph;
      MTSP_grid_graph_.last_robot_position_[itr_robot.first] = itr_robot.second[i];
    }
  }
  if (MTSP_grid_graph_.last_robot_position_.count(cur_subgraph.robot_id) == 0)  //不存在  就初始化
  {
    geometry_msgs::msg::Point temp;
    temp.x = 0;
    temp.y = 0;
    temp.z = 0;
    MTSP_grid_graph_.last_robot_position_.insert(std::pair<int, geometry_msgs::msg::Point>(cur_subgraph.robot_id, temp));
    int temp_node_index = -1;
    MTSP_grid_graph_.last_robot_node_index_.insert(std::pair<int, int>(cur_subgraph.robot_id, temp_node_index));
  }
  //是否  添加点  （添加点就必然添加边） 到  MTSP_grid_graph
  double distance = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
      MTSP_grid_graph_.last_robot_position_[cur_subgraph.robot_id], cur_subgraph.robot_postion);
  if (distance > kAddNodeMinDist)  //距离小于最小添加距离  就不加入,  大于就加入并添加边
  {
    //添加点
    int robot_index_MTSP_grid_graph;
    double dist;
    //当前点和连通树   在   距离小于    kAddNodeMinDist  中存在点  就不添加点而直接使用查找到的点
    // std::cout<<"查询是否存在  范围内的  连通结点  "<<std::endl;
    bool is_range_exist = MTSP_grid_graph_.GetCloseConnectNodeMtspGraph(
        cur_subgraph.robot_postion, robot_index_MTSP_grid_graph, dist, kAddNodeMinDist);
    if (!is_range_exist)  //不存在添加点  存在就不添加点
    {
      robot_index_MTSP_grid_graph = MTSP_grid_graph_.MTSP_graph_nodes_.size();
      int robot_cell_id =
          GetCellInd(cur_subgraph.robot_postion.x, cur_subgraph.robot_postion.y, cur_subgraph.robot_postion.z);
      MTSP_grid_graph_.AddNode(cur_subgraph.robot_id, robot_cell_id, cur_subgraph.robot_postion, true,
                               MTSP_grid_graph_.MTSP_graph_nodes_.size());
    }
    MTSP_grid_graph_.MTSP_graph_nodes_[robot_index_MTSP_grid_graph].is_keypose_ = true;

    //  给探索点  分配  发现时的机器人位置索引
    for (int k = 0; k < cur_subgraph.neighbor_nodes.size(); k++)
    {
      int cell_index = cur_subgraph.neighbor_nodes[k].cell_index;
      if (cur_subgraph.neighbor_nodes[k].cell_statu == 1 &&
          subspaces_world_->GetCell(cell_index).GetMTSP_graph_index_find_exploring_cell_() == -1)
      {
        subspaces_world_->GetCell(cell_index).SetMTSP_graph_index_find_exploring_cell_(robot_index_MTSP_grid_graph);
      }
    }

    //是否添加边
    if (MTSP_grid_graph_.last_robot_node_index_[cur_subgraph.robot_id] != -1)  //上一个为-1  则是初始化
                                                                               //跳过，只添加点即可
    {
      //添加与前一个路经点的 边
      //判定是否存在边
      if (!MTSP_grid_graph_.is_edge_exist(MTSP_grid_graph_.last_robot_node_index_[cur_subgraph.robot_id],
                                          robot_index_MTSP_grid_graph))
      {
        MTSP_grid_graph_.AddEdge(MTSP_grid_graph_.last_robot_node_index_[cur_subgraph.robot_id],
                                 robot_index_MTSP_grid_graph, distance);
      }
    }
    ConnectRobotToNeighborCellGraphNode_local_pro(robot_index_MTSP_grid_graph, 2.5 * kCellSize, planning_env_,
                                                  kAddEdgeCollisionCheckResolution, viewpoint_manager_);

    //更新  最后点
    MTSP_grid_graph_.last_robot_node_index_[cur_subgraph.robot_id] = robot_index_MTSP_grid_graph;
    MTSP_grid_graph_.last_robot_position_[cur_subgraph.robot_id] = cur_subgraph.robot_postion;
  }
  //清空历史位置   及单元数据
  for (auto& itr_robot : other_robot_history_position_)
  {
    other_robot_history_position_[itr_robot.first].clear();
    robot_cell_subnode_[itr_robot.first].clear();
  }
}
void GridWorld::ConnectRobotToNeighborCellGraphNode(int robot_index_MTSP_grid_graph, double kAddEdgeConnectDistThr,
                                                    const std::unique_ptr<planning_env_ns::PlanningEnv>& planning_env_,
                                                    double kAddEdgeCollisionCheckResolution)
{
  MTSP_graph_node temp = MTSP_grid_graph_.MTSP_graph_nodes_[robot_index_MTSP_grid_graph];
  //查询  6个  距离为1  的网格
  Eigen::Vector3i from_cell_sub = subspaces_->Ind2Sub(temp.cell_id_);  //单元ID 编码    转子空间  三维坐标
  std::vector<int> nearby_cell_indices;  //   from_cell  的  邻接的单元ID    本地单元+     距离为1的  6个邻接  单元
                                         //   一共7个
  nearby_cell_indices.push_back(temp.cell_id_);  //加入当前 cell
  //空间上 周围    27个的 单元   邻接
  for (int x = -1; x <= 1; x++)
  {
    for (int y = -1; y <= 1; y++)
    {
      for (int z = -1; z <= 1; z++)
      {
        if (std::abs(x) + std::abs(y) + std::abs(z) == 1)  //距离为1的  6个邻接  单元
        {
          Eigen::Vector3i neighbor_sub = from_cell_sub + Eigen::Vector3i(x, y, z);  //以起点  单元  ID  为起始
          // if (SubInBound(neighbor_sub))
          if (subspaces_->InRange(neighbor_sub))  //在子空间范围内
          {
            // int neighbor_ind = sub2ind(neighbor_sub);
            int neighbor_ind = subspaces_->Sub2Ind(neighbor_sub);
            nearby_cell_indices.push_back(neighbor_ind);
          }
        }
      }
    }
  }
  //   nearby_cell_indices 包含7个网格   1 个本地  + 6个邻接网格
  for (int i = 0; i < MTSP_grid_graph_.MTSP_graph_nodes_.size(); i++)
  {
    if (i == robot_index_MTSP_grid_graph)  //  跳过当前位置
    {
      continue;
    }
    int cell_index = MTSP_grid_graph_.MTSP_graph_nodes_[i].cell_id_;
    for (int j = 0; j < nearby_cell_indices.size(); j++)  //查询是否是这七个中的点
    {
      if (cell_index == nearby_cell_indices[j] &&  //是七个邻接网格中的点
          (subspaces_world_->GetCell(cell_index).GetStatus() == CellStatus::COVERED ||
           subspaces_world_->GetCell(cell_index).GetStatus() == CellStatus::COVERED_BY_OTHERS) &&  //是已探索状态
          MTSP_grid_graph_.graph_[i].size() > 0 &&  //是连通的点才能加边
          !(MTSP_grid_graph_.MTSP_graph_nodes_[i].robot_id_ ==
                MTSP_grid_graph_.MTSP_graph_nodes_[robot_index_MTSP_grid_graph].robot_id_ &&
            MTSP_grid_graph_.MTSP_graph_nodes_[i].is_keypose_))  //不能是当前的 历史路经
      {
        IsAddExtraEdge(robot_index_MTSP_grid_graph, i, kAddEdgeConnectDistThr, planning_env_,
                       kAddEdgeCollisionCheckResolution);
        break;
      }
    }
  }
}

//新增  激进的  当前机器人添加  额外边
void GridWorld::ConnectRobotToNeighborCellGraphNode_local_pro(
    int robot_index_MTSP_grid_graph, double kAddEdgeConnectDistThr,
    const std::unique_ptr<planning_env_ns::PlanningEnv>& planning_env_, double kAddEdgeCollisionCheckResolution,
    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager_)
{
  MTSP_graph_node temp = MTSP_grid_graph_.MTSP_graph_nodes_[robot_index_MTSP_grid_graph];
  //查询  6个  距离为1  的网格
  Eigen::Vector3i from_cell_sub = subspaces_->Ind2Sub(temp.cell_id_);  //单元ID 编码    转子空间  三维坐标
  std::vector<int> nearby_cell_indices;  //   from_cell  的  邻接的单元ID    本地单元+     距离为1的  6个邻接  单元
                                         //   一共7个
  // nearby_cell_indices.push_back(temp.cell_id_);//加入当前 cell
  //空间上 周围    27个的 单元   邻接
  for (int x = -1; x <= 1; x++)
  {
    for (int y = -1; y <= 1; y++)
    {
      for (int z = -1; z <= 1; z++)
      {
        if (std::abs(x) + std::abs(y) + std::abs(z) <= 3)  // 全部邻接单元
        {
          Eigen::Vector3i neighbor_sub = from_cell_sub + Eigen::Vector3i(x, y, z);  //以起点  单元  ID  为起始
          // if (SubInBound(neighbor_sub))
          if (subspaces_->InRange(neighbor_sub))  //在子空间范围内
          {
            // int neighbor_ind = sub2ind(neighbor_sub);
            int neighbor_ind = subspaces_->Sub2Ind(neighbor_sub);
            nearby_cell_indices.push_back(neighbor_ind);
          }
        }
      }
    }
  }
  //   nearby_cell_indices 包含7个网格   1 个本地  + 6个邻接网格
  for (int i = 0; i < MTSP_grid_graph_.MTSP_graph_nodes_.size(); i++)
  {
    if (i == robot_index_MTSP_grid_graph)  //  跳过当前位置
    {
      continue;
    }
    int cell_index = MTSP_grid_graph_.MTSP_graph_nodes_[i].cell_id_;
    for (int j = 0; j < nearby_cell_indices.size(); j++)  //查询是否是这七个中的点
    {
      if (cell_index == nearby_cell_indices[j] &&  //是七个邻接网格中的点
          (subspaces_world_->GetCell(cell_index).GetStatus() == CellStatus::COVERED ||
           subspaces_world_->GetCell(cell_index).GetStatus() == CellStatus::COVERED_BY_OTHERS) &&  //是已探索状态
          MTSP_grid_graph_.graph_[i].size() > 0 &&  //是连通的点才能加边
          !(MTSP_grid_graph_.MTSP_graph_nodes_[i].robot_id_ ==
                MTSP_grid_graph_.MTSP_graph_nodes_[robot_index_MTSP_grid_graph].robot_id_ &&
            MTSP_grid_graph_.MTSP_graph_nodes_[i].is_keypose_))  //不能是当前的 历史路经
      {
        IsAddExtraEdge_local_pro(robot_index_MTSP_grid_graph, i, kAddEdgeConnectDistThr, planning_env_,
                                 kAddEdgeCollisionCheckResolution, viewpoint_manager_);
        break;
      }
    }
  }
}

//检测是否  添加 额外边对于  两个结点  在  MTSP_grid_graph  上
void GridWorld::IsAddExtraEdge(int from_node_ind, int to_node_ind, double MinDistThr,
                               const std::unique_ptr<planning_env_ns::PlanningEnv>& planning_env_,
                               double kAddEdgeCollisionCheckResolution)
{
  bool is_add = false;
  bool is_neighbor_cell_from = false;  //需要增加边    的起始点是否在邻接范围内
  bool is_neighbor_cell_to = false;    //需要增加边     的终点是否在连接范围内
  bool in_collision = false;
  bool is_short = false;
  bool in_blacklist_from = false;
  bool in_blacklist_to = false;
  bool is_be = MTSP_grid_graph_.is_edge_exist(from_node_ind, to_node_ind);  //是否已经存在  两个结点的边
  int from_cell_index = MTSP_grid_graph_.MTSP_graph_nodes_[from_node_ind].cell_id_;
  int to_cell_index = MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].cell_id_;
  geometry_msgs::msg::Point from_node_position = MTSP_grid_graph_.MTSP_graph_nodes_[from_node_ind].position_;
  geometry_msgs::msg::Point to_node_position = MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].position_;
  double distance_ = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
      MTSP_grid_graph_.MTSP_graph_nodes_[from_node_ind].position_,
      MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].position_);
  if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), from_cell_index) !=
      neighbor_cell_indices_.end())
  {
    is_neighbor_cell_from = true;
  }
  if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), to_cell_index) !=
      neighbor_cell_indices_.end())
  {
    is_neighbor_cell_to = true;
  }
  //两个都在局部规划框内      就做一下   判断是否有障碍
  if (is_neighbor_cell_from && is_neighbor_cell_to)
  {
    double neighbor_node_dist = distance_;
    double diff_x = from_node_position.x - to_node_position.x;
    double diff_y = from_node_position.y - to_node_position.y;
    double diff_z = from_node_position.z - to_node_position.z;
    int check_point_num = static_cast<int>(neighbor_node_dist / kAddEdgeCollisionCheckResolution);  //检测点数量
    for (int i = 0; i < check_point_num; i++)
    {
      // std::cout << "checking the " << i << " collision point" << std::endl;
      double check_point_x = from_node_position.x + kAddEdgeCollisionCheckResolution * i * diff_x / neighbor_node_dist;
      double check_point_y = from_node_position.y + kAddEdgeCollisionCheckResolution * i * diff_y / neighbor_node_dist;
      double check_point_z = from_node_position.z + kAddEdgeCollisionCheckResolution * i * diff_z / neighbor_node_dist;
      if (planning_env_->InCollision(check_point_x, check_point_y, check_point_z))  //将检测点  放入  环境类中
                                                                                    //检测是否在碰撞
      {
        in_collision = true;
        break;
      }
    }
    if (in_collision)  //当两个点有障碍就  添加黑名单
    {
      MTSP_grid_graph_.MTSP_graph_nodes_[from_node_ind].blacklist_node_index.push_back(to_node_ind);
      MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].blacklist_node_index.push_back(from_node_ind);
      if (is_be)  //存在两个结点的边  就要删除这条边
      {
        MTSP_grid_graph_.DelEdge(from_node_ind, to_node_ind);
      }
    }
  }

  //查询下黑名单
  if (!in_collision)  //非碰撞查   黑名单
  {
    std::vector<int> from_blacklist = MTSP_grid_graph_.MTSP_graph_nodes_[from_node_ind].blacklist_node_index;
    std::vector<int> to_blacklist = MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].blacklist_node_index;
    if (std::find(from_blacklist.begin(), from_blacklist.end(), to_node_ind) != from_blacklist.end())  //  在  from
                                                                                                       //  的查询  to
    {
      in_blacklist_to = true;
    }
    if (std::find(to_blacklist.begin(), to_blacklist.end(), from_node_ind) != to_blacklist.end())
    {
      in_blacklist_from = true;
    }
  }
  //查询 z轴高度，z轴相距小于
  double z_dist = std::abs(MTSP_grid_graph_.MTSP_graph_nodes_[from_node_ind].position_.z -
                           MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].position_.z);
  if (distance_ < MinDistThr && z_dist < kCellHeight / 3)  //距离小于   阈值  kAddNodeMinDist   则将两个子图给连起来
                                                           //连线
  {
    is_short = true;
  }

  //连接情况：1在局部网格内  非碰撞   就连接  不考虑  长短    不存在边
  //连接情况：2不在局部网格内  非黑名单  距离小于  阈值 就连接  不存在边
  bool is_be_ = MTSP_grid_graph_.is_edge_exist(from_node_ind, to_node_ind);
  if (is_neighbor_cell_from && is_neighbor_cell_to && !in_collision && !is_be_ && is_short)
  {
    // std::cout<<"添加额外边   "<<from_node_ind<<"  to "<<to_node_ind <<std::endl;
    MTSP_grid_graph_.AddEdge(from_node_ind, to_node_ind, distance_);
  }
  else if (!(is_neighbor_cell_from && is_neighbor_cell_to) && !in_blacklist_to && !in_blacklist_from && is_short &&
           !is_be_)
  {
    // std::cout<<"添加额外边   "<<from_node_ind<<"  to "<<to_node_ind <<std::endl;
    MTSP_grid_graph_.AddEdge(from_node_ind, to_node_ind, distance_);
  }
}

//新增  是否添额外边
void GridWorld::IsAddExtraEdge_local_pro(
    int from_node_ind, int to_node_ind, double MinDistThr,
    const std::unique_ptr<planning_env_ns::PlanningEnv>& planning_env_, double kAddEdgeCollisionCheckResolution,
    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager_)
{
  bool is_add = false;
  bool is_neighbor_cell_from = false;  //需要增加边    的起始点是否在邻接范围内
  bool is_neighbor_cell_to = false;    //需要增加边     的终点是否在连接范围内
  bool in_collision = false;
  bool is_short = false;
  bool in_blacklist_from = false;
  bool in_blacklist_to = false;
  bool in_Sight = false;
  bool is_be = MTSP_grid_graph_.is_edge_exist(from_node_ind, to_node_ind);  //是否已经存在  两个结点的边
  int from_cell_index = MTSP_grid_graph_.MTSP_graph_nodes_[from_node_ind].cell_id_;
  int to_cell_index = MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].cell_id_;
  geometry_msgs::msg::Point from_node_position = MTSP_grid_graph_.MTSP_graph_nodes_[from_node_ind].position_;
  geometry_msgs::msg::Point to_node_position = MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].position_;

  // 查询  to_node_ind   是否候选视点
  Eigen::Vector3d to_node_position_v(to_node_position.x, to_node_position.y, to_node_position.z);
  int to_node_position_index_in_viewpoint_manager = viewpoint_manager_->GetViewPointInd(to_node_position_v);
  if (to_node_position_index_in_viewpoint_manager == -1)
  {
    return;
  }
  if (!viewpoint_manager_->ViewPointInCollision(to_node_position_index_in_viewpoint_manager) &&
      viewpoint_manager_->ViewPointInCurrentFrameLineOfSight(to_node_position_index_in_viewpoint_manager) &&
      viewpoint_manager_->ViewPointConnected(to_node_position_index_in_viewpoint_manager))
  {
    in_Sight = true;
  }

  double distance_ = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
      MTSP_grid_graph_.MTSP_graph_nodes_[from_node_ind].position_,
      MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].position_);
  if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), from_cell_index) !=
      neighbor_cell_indices_.end())
  {
    is_neighbor_cell_from = true;
  }
  if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), to_cell_index) !=
      neighbor_cell_indices_.end())
  {
    is_neighbor_cell_to = true;
  }
  //两个都在局部规划框内      就做一下   判断是否有障碍
  if (is_neighbor_cell_from && is_neighbor_cell_to)
  {
    double neighbor_node_dist = distance_;
    double diff_x = from_node_position.x - to_node_position.x;
    double diff_y = from_node_position.y - to_node_position.y;
    double diff_z = from_node_position.z - to_node_position.z;
    int check_point_num = static_cast<int>(neighbor_node_dist / kAddEdgeCollisionCheckResolution);  //检测点数量
    for (int i = 0; i < check_point_num; i++)
    {
      // std::cout << "checking the " << i << " collision point" << std::endl;
      double check_point_x = from_node_position.x + kAddEdgeCollisionCheckResolution * i * diff_x / neighbor_node_dist;
      double check_point_y = from_node_position.y + kAddEdgeCollisionCheckResolution * i * diff_y / neighbor_node_dist;
      double check_point_z = from_node_position.z + kAddEdgeCollisionCheckResolution * i * diff_z / neighbor_node_dist;
      if (planning_env_->InCollision(check_point_x, check_point_y, check_point_z))  //将检测点  放入  环境类中
                                                                                    //检测是否在碰撞
      {
        in_collision = true;
        break;
      }
    }
    if (in_collision)  //当两个点有障碍就  添加黑名单
    {
      MTSP_grid_graph_.MTSP_graph_nodes_[from_node_ind].blacklist_node_index.push_back(to_node_ind);
      MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].blacklist_node_index.push_back(from_node_ind);
      if (is_be)  //存在两个结点的边  就要删除这条边
      {
        MTSP_grid_graph_.DelEdge(from_node_ind, to_node_ind);
      }
    }
  }

  //查询下黑名单
  if (!in_collision)  //非碰撞查   黑名单
  {
    std::vector<int> from_blacklist = MTSP_grid_graph_.MTSP_graph_nodes_[from_node_ind].blacklist_node_index;
    std::vector<int> to_blacklist = MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].blacklist_node_index;
    if (std::find(from_blacklist.begin(), from_blacklist.end(), to_node_ind) != from_blacklist.end())  //  在  from
                                                                                                       //  的查询  to
    {
      in_blacklist_to = true;
    }
    if (std::find(to_blacklist.begin(), to_blacklist.end(), from_node_ind) != to_blacklist.end())
    {
      in_blacklist_from = true;
    }
  }
  //查询 z轴高度，z轴相距小于
  double z_dist = std::abs(MTSP_grid_graph_.MTSP_graph_nodes_[from_node_ind].position_.z -
                           MTSP_grid_graph_.MTSP_graph_nodes_[to_node_ind].position_.z);
  if (distance_ < MinDistThr && z_dist < kCellHeight / 3)  //距离小于   阈值  kAddNodeMinDist   则将两个子图给连起来
                                                           //连线
  {
    is_short = true;
  }

  //连接情况：1在局部网格内  非碰撞   就连接  不考虑  长短    不存在边
  //连接情况：2不在局部网格内  非黑名单  距离小于  阈值 就连接  不存在边
  bool is_be_ = MTSP_grid_graph_.is_edge_exist(from_node_ind, to_node_ind);
  if (is_neighbor_cell_from && is_neighbor_cell_to && !in_collision && !is_be_ && is_short && in_Sight)
  {
    // std::cout<<"添加额外边   "<<from_node_ind<<"  to "<<to_node_ind <<std::endl;
    MTSP_grid_graph_.AddEdge(from_node_ind, to_node_ind, distance_);
  }
  else if (!(is_neighbor_cell_from && is_neighbor_cell_to) && !in_blacklist_to && !in_blacklist_from && is_short &&
           !is_be_ && in_Sight)
  {
    // std::cout<<"添加额外边   "<<from_node_ind<<"  to "<<to_node_ind <<std::endl;
    MTSP_grid_graph_.AddEdge(from_node_ind, to_node_ind, distance_);
  }
}

//新增MTSP_grid_graph 可视化

void GridWorld::GetGridGraphMarker(visualization_msgs::msg::Marker& node_marker, visualization_msgs::msg::Marker& edge_marker)
{
  node_marker.points.clear();
  edge_marker.points.clear();
  if (MTSP_grid_graph_.MTSP_graph_nodes_.empty())
  {
    return;
  }

  for (const auto& node : MTSP_grid_graph_.MTSP_graph_nodes_)
  {
    node_marker.points.push_back(node.position_);
  }

  std::vector<std::pair<int, int>> added_edge;
  for (int i = 0; i < MTSP_grid_graph_.graph_.size(); i++)
  {
    int start_ind = i;
    for (int j = 0; j < MTSP_grid_graph_.graph_[i].size(); j++)
    {
      int end_ind = MTSP_grid_graph_.graph_[i][j];
      if (std::find(added_edge.begin(), added_edge.end(), std::make_pair(start_ind, end_ind)) == added_edge.end())
      {
        geometry_msgs::msg::Point start_node_position = MTSP_grid_graph_.MTSP_graph_nodes_[start_ind].position_;
        geometry_msgs::msg::Point end_node_position = MTSP_grid_graph_.MTSP_graph_nodes_[end_ind].position_;
        edge_marker.points.push_back(start_node_position);
        edge_marker.points.push_back(end_node_position);
        added_edge.emplace_back(start_ind, end_ind);
      }
    }
  }
}

//新增MTSP_grid_graph 可视化

void GridWorld::GetVisualizationGridGraphCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  cloud->clear();
  for (const auto& node : MTSP_grid_graph_.MTSP_graph_nodes_)
  {
    pcl::PointXYZI point;
    point.x = node.position_.x;
    point.y = node.position_.y;
    point.z = node.position_.z;
    if (node.is_connected_)
    {
      point.intensity = 10;  //连通的强度10
    }
    else
    {
      point.intensity = -1;  //非连通 强度-1
    }
    cloud->points.push_back(point);
  }
}

//检测碰撞   试一下
void GridWorld::CheckGridGraphLocalCollision(
    const geometry_msgs::msg::Point& robot_position,
    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
{
  // Get local planning horizon xy size     得到本地规划 范围 xy  大小
  int in_local_planning_horizon_count = 0;                                               //在局部规划的数量
  int collision_node_count = 0;                                                          //障碍点数
  int collision_edge_count = 0;                                                          //障碍边数
  int in_viewpoint_range_count = 0;                                                      //视点范围数
  Eigen::Vector3d viewpoint_resolution = viewpoint_manager->GetResolution();             //视点分辨率
  double max_z_diff = std::max(viewpoint_resolution.x(), viewpoint_resolution.y()) * 2;  //高度差  0.5*2=   1m
  std::vector<MTSP_graph_node> nodes_ = MTSP_grid_graph_.MTSP_graph_nodes_;
  for (int i = 0; i < nodes_.size(); i++)  //对之前存在的结点
  {
    //检测属性  ，只检测        之前  额外添加的边
    // 1、同cell里面的  不同机器人的    road_map_point  相互连接  额外添加
    // 2、
    if (nodes_[i].is_keypose_)
    {
      continue;
    }
    //当不是关键位姿结点
    Eigen::Vector3d node_position =
        Eigen::Vector3d(nodes_[i].position_.x, nodes_[i].position_.y, nodes_[i].position_.z);
    int viewpoint_ind = viewpoint_manager->GetViewPointInd(node_position);  //当前这个结点  在局部  视点集里 编号
                                                                            //用局部信息  视点编号操作
    bool node_in_collision = false;  //初始化默认不在  障碍里面
    if (viewpoint_manager->InRange(viewpoint_ind) &&
        std::abs(viewpoint_manager->GetViewPointHeight(viewpoint_ind) - node_position.z()) <
            max_z_diff)  //在局部规划框范围里  才更新     且 高度在阈值  1m 以内
    {
      in_local_planning_horizon_count++;  //在局部规划范围内  增加计数
      in_viewpoint_range_count++;
      if (viewpoint_manager->ViewPointInCollision(viewpoint_ind))  //当前视点   在  碰撞里
      {
        node_in_collision = true;
        collision_node_count++;
        // Delete all the associated edges   删除所有关联的边    在障碍物里面就删除所以相关联的边
        for (int j = 0; j < MTSP_grid_graph_.graph_[i].size(); j++)
        {
          int neighbor_ind = MTSP_grid_graph_.graph_[i][j];
          for (int k = 0; k < MTSP_grid_graph_.graph_[neighbor_ind].size(); k++)
          {
            if (MTSP_grid_graph_.graph_[neighbor_ind][k] == i)
            {
              MTSP_grid_graph_.graph_[neighbor_ind].erase(MTSP_grid_graph_.graph_[neighbor_ind].begin() + k);
              MTSP_grid_graph_.dist_[neighbor_ind].erase(MTSP_grid_graph_.dist_[neighbor_ind].begin() + k);
              k--;
            }
          }
        }
        MTSP_grid_graph_.graph_[i].clear();
        MTSP_grid_graph_.dist_[i].clear();
      }
      else  //当前结点不碰撞
      {
        Eigen::Vector3d viewpoint_resolution = viewpoint_manager->GetResolution();  //视点分辨率
        //修改  不需要  变成  二分之一
        // double collision_check_resolution = std::min(viewpoint_resolution.x(), viewpoint_resolution.y()) /
        // 2;//0.5/2=0.25
        double collision_check_resolution = std::min(viewpoint_resolution.x(), viewpoint_resolution.y());
        // Check edge collision     检测边碰撞
        for (int j = 0; j < MTSP_grid_graph_.graph_[i].size(); j++)
        {
          int neighbor_ind = MTSP_grid_graph_.graph_[i][j];

          Eigen::Vector3d start_position = node_position;
          Eigen::Vector3d end_position = Eigen::Vector3d(
              nodes_[neighbor_ind].position_.x, nodes_[neighbor_ind].position_.y, nodes_[neighbor_ind].position_.z);
          int viewpoint_ind_end = viewpoint_manager->GetViewPointInd(end_position);
          if (!(viewpoint_manager->InRange(viewpoint_ind_end) &&
                std::abs(viewpoint_manager->GetViewPointHeight(viewpoint_ind_end) - node_position.z()) < max_z_diff))
          {
            continue;
          }
          //边的两个端点都得在局部规划框内  才能检测   并  删除边
          std::vector<Eigen::Vector3d> interp_points;
          misc_utils_ns::LinInterpPoints(start_position, end_position, collision_check_resolution,
                                         interp_points);  //两个结点 ， 碰撞检测分辨率， 线性内部点集合 interp_points
          for (const auto& collision_check_position : interp_points)  //检测这些线性内部点集合
          {
            int viewpoint_ind = viewpoint_manager->GetViewPointInd(collision_check_position);  //得到  线性内部点 的ID
            if (viewpoint_manager->InRange(viewpoint_ind))  //判断  此ID  是否在范围内
            {
              if (viewpoint_manager->ViewPointInCollision(viewpoint_ind))  //判断这个ID   是不是在障碍内
              {
                geometry_msgs::msg::Point viewpoint_position =
                    viewpoint_manager->GetViewPointPosition(viewpoint_ind);  //得到这个视点的  位置
                // Delete neighbors' edges    删除 连接的边
                //  删除边 是否需要  加入黑名单？
                //删除边  加入 黑名单

                for (int k = 0; k < MTSP_grid_graph_.graph_[neighbor_ind].size(); k++)
                {
                  if (MTSP_grid_graph_.graph_[neighbor_ind][k] == i)
                  {
                    collision_edge_count++;
                    MTSP_grid_graph_.graph_[neighbor_ind].erase(MTSP_grid_graph_.graph_[neighbor_ind].begin() + k);
                    MTSP_grid_graph_.dist_[neighbor_ind].erase(MTSP_grid_graph_.dist_[neighbor_ind].begin() + k);
                    MTSP_grid_graph_.MTSP_graph_nodes_[neighbor_ind].blacklist_node_index.push_back(i);
                    k--;
                  }
                }
                // Delete the node's edge   删除这个结点的边
                MTSP_grid_graph_.graph_[i].erase(MTSP_grid_graph_.graph_[i].begin() + j);
                MTSP_grid_graph_.dist_[i].erase(MTSP_grid_graph_.dist_[i].begin() + j);
                MTSP_grid_graph_.MTSP_graph_nodes_[i].blacklist_node_index.push_back(neighbor_ind);
                j--;
                break;
              }
            }
          }
        }
      }
    }
  }
}
//检测连通性
void GridWorld::CheckGridGraphConnectivity(const geometry_msgs::msg::Point& robot_position)
{
  if (MTSP_grid_graph_.MTSP_graph_nodes_.empty())
  {
    return;
  }
  std::cout << "MTSP_graph  更新结点" << std::endl;
  MTSP_grid_graph_.UpdateNodes();  //更新结点

  // The first keypose node is always connected, set all the others to be disconnected      第一个关键位姿即诶单时相连的
  // ，设置其他的不连通
  int first_keypose_node_ind = -1;
  bool found_connected = false;

  for (int i = 0; i < MTSP_grid_graph_.MTSP_graph_nodes_.size(); i++)
  {
    if (MTSP_grid_graph_.MTSP_graph_nodes_[i].is_keypose_ &&
        MTSP_grid_graph_.MTSP_graph_nodes_[i].robot_id_ == cur_robot_id_)  //当找到是关键位姿的结点  就跳出循环
                                                                           //机器人行走过的位置肯定是连通的
    {
      first_keypose_node_ind = i;
      break;
    }
  }
  std::cout << "MTSP_graph  找到机器人位置点" << std::endl;
  // Check the connectivity starting from the robot    机器人开始前  检测是否连通
  for (int i = 0; i < MTSP_grid_graph_.MTSP_graph_nodes_.size(); i++)  //初始化  所有结点 都   不连通
  {
    MTSP_grid_graph_.MTSP_graph_nodes_[i].is_connected_ = false;
  }

  if (first_keypose_node_ind >= 0 &&
      first_keypose_node_ind < MTSP_grid_graph_.MTSP_graph_nodes_.size())  //存在关键位姿结点
  {
    MTSP_grid_graph_.MTSP_graph_nodes_[first_keypose_node_ind].is_connected_ = true;  //第一个  是航迹点  肯定是连接的
    MTSP_grid_graph_.connected_node_indices_.clear();                                 //清空连接点集
    std::vector<bool> constraint(MTSP_grid_graph_.MTSP_graph_nodes_.size(), true);  //全部设置为  true
    std::cout << "MTSP_graph  得到连接的结点" << std::endl;
    MTSP_grid_graph_.GetConnectedNodeIndices(first_keypose_node_ind, MTSP_grid_graph_.connected_node_indices_,
                                             constraint);  //得到连接的结点  ID
  }
  else  //不存在关键位姿结点         即才初始化
  {
    int robot_node_ind = -1;
    double robot_node_dist = DBL_MAX;
    std::cout << "MTSP_graph  得到距离机器人最近的结点" << std::endl;
    MTSP_grid_graph_.GetClosestNodeIndAndDistance(robot_position, robot_node_ind,
                                                  robot_node_dist);  //得到距离  机器人位置     最近结点
                                                                     //的关键位姿图上的 结点id 和距离
    if (robot_node_ind >= 0 && robot_node_ind < MTSP_grid_graph_.MTSP_graph_nodes_.size())
    {
      MTSP_grid_graph_.MTSP_graph_nodes_[robot_node_ind].is_connected_ = true;
      MTSP_grid_graph_.connected_node_indices_.clear();
      std::vector<bool> constraint(MTSP_grid_graph_.MTSP_graph_nodes_.size(), true);
      std::cout << "MTSP_graph  得到连接的结点" << std::endl;
      MTSP_grid_graph_.GetConnectedNodeIndices(robot_node_ind, MTSP_grid_graph_.connected_node_indices_,
                                               constraint);  //得到与  robot_node_ind  相连的  ID
    }
    else
    {
      // ROS_ERROR_STREAM("KeyposeGraph::CheckConnectivity: Cannot get closest robot node ind "
      //                  << robot_node_ind);  //不能得到最近的机器人结点 ID
    }
  }

  MTSP_grid_graph_.connected_nodes_cloud_->clear();
  std::cout << "MTSP_graph  扫描连接的点" << std::endl;
  for (int i = 0; i < MTSP_grid_graph_.connected_node_indices_.size(); i++)  //找到连接的关键位姿图就   更新
  {
    int node_ind = MTSP_grid_graph_.connected_node_indices_[i];
    MTSP_grid_graph_.MTSP_graph_nodes_[node_ind].is_connected_ = true;
    pcl::PointXYZI point;
    point.x = MTSP_grid_graph_.MTSP_graph_nodes_[node_ind].position_.x;
    point.y = MTSP_grid_graph_.MTSP_graph_nodes_[node_ind].position_.y;
    point.z = MTSP_grid_graph_.MTSP_graph_nodes_[node_ind].position_.z;
    point.intensity = node_ind;  //强度就是结点的  index
    MTSP_grid_graph_.connected_nodes_cloud_->points.push_back(point);
  }
  if (!MTSP_grid_graph_.connected_nodes_cloud_->points.empty())
  {
    MTSP_grid_graph_.kdtree_connected_nodes_->setInputCloud(MTSP_grid_graph_.connected_nodes_cloud_);
  }
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

//得到 做MTSP  信息
bool GridWorld::GetMtspGridGraphInformation(
    std::vector<int>& exploring_cell_indices, std::vector<geometry_msgs::msg::Point>& exploring_cell_positions,
    std::vector<std::vector<int>>& distance_matrix,
    std::vector<int>& exploring_cell_indices_MTSP,                     //   做MTSP  的  探索单元  ID
    std::vector<geometry_msgs::msg::Point>& exploring_cell_positions_MTSP,  //做 MTSP  探索单元  ID
    std::vector<std::vector<int>>& distance_matrix_MTSP,               //  做MTSP 的距离矩阵
    std::vector<int>& robot_position_id_on_exploring_cell_positions_MTSP,
    std::map<int, std::vector<double>>& MTSP_subgrapher_map_,
    const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
    std::vector<int>& positions_MTSP_grid_graph_node_id)
{
  //解码     MTSP_subgrapher_map_
  std::map<int, std::vector<geometry_msgs::msg::Point>> robot_ID_positions_map;
  std::map<int, std::vector<std::vector<int>>> robot_ID_dis_matrix_map;
  std::map<int, std::vector<geometry_msgs::msg::Point>> robot_ID_positions_on_keyposegrapher;  //外部探索点   在本地可到达
                                                                                          //已探索点
  std::map<int, std::vector<int>>
      robot_ID_positions_ID_on_keyposegrapher;  //外部探索点   在本地可到达  已探索点
                                                //<机器人编号,外部探索点在位置向量中的索引ID>
  std::map<int, std::vector<geometry_msgs::msg::Point>> robot_ID_positions_out_keyposegrapher;  //外部探索点   在本地不可到达
                                                                                           //未知探索点
  std::map<int, std::vector<int>>
      robot_ID_positions_ID_out_keyposegrapher;  //外部探索点   在本地不可到达  未知探索点
                                                 //<机器人编号,外部探索点在位置向量中的索引ID>
  std::map<int, std::vector<std::vector<int>>> Cell_ID_positions_global;  //用于做  MTSP的  单元  ID 和位置。
  std::map<int, bool> robot_ID_is_fuse;                                   //<机器人ID ， 是否融合成功 >
  int other_submtspgrapher_num;
  other_submtspgrapher_num = MTSP_subgrapher_map_.size();
  if (other_submtspgrapher_num > 0)  //当有外部输入  子图
  {
    //对每个外部输入的子图       解码   得到  robot_ID_positions_map   robot_ID_dis_matrix_map  两个字典
    // std::cout << std::endl << "\033[1;32m" << "解码生成子图字典，并存储" << "\033[0m" << std::endl;
    for (auto itr : MTSP_subgrapher_map_)
    {
      robot_ID_is_fuse.insert(std::pair<int, bool>(itr.first, false));  //对每个外部子图都创建一个判定  初始化为   false
      int robot_id;
      robot_id = itr.first;
      int node_num = itr.second[0];  // 0  位是结点数量
      int count = 1;
      std::vector<geometry_msgs::msg::Point> positions;
      std::vector<std::vector<int>> dis_matrix(node_num, std::vector<int>(node_num, 0));

      geometry_msgs::msg::Point point_;
      for (int i = 1; i < itr.second.size(); i++)  //  3*n    是机器人位置信息   n*n  是距离矩阵
      {
        if (i <= 3 * node_num)  // 3*n
        {
          if (i % 3 == 1)
          {
            point_.x = itr.second[i];
          }
          else if (i % 3 == 2)
          {
            point_.y = itr.second[i];
          }
          else if (i % 3 == 0)
          {
            point_.z = itr.second[i];
            positions.push_back(point_);
          }
        }
        else
        {  //  构造   距离矩阵      n*n
          int node_i = (i - 3 * node_num - 1) / node_num;
          int node_j = (i - 3 * node_num - 1) % node_num;
          dis_matrix[node_i][node_j] = (int)itr.second[i];
        }
      }
      robot_ID_positions_map.insert(std::pair<int, std::vector<geometry_msgs::msg::Point>>(itr.first, positions));
      robot_ID_dis_matrix_map.insert(std::pair<int, std::vector<std::vector<int>>>(itr.first, dis_matrix));
    }

    //打印   本地子图个数 及结点  编号

    //打印接受子图个数   及结点编号

    //使用  robot_ID_positions_map   和    robot_ID_dis_matrix_map    进行融合
    //  使用     robot_ID_positions_map   分类   目标点     在关键位姿图上（已探索目标点）
    //  不在关键位姿图上（未探索区域）
    //得到  分类位置  索引  robot_ID_positions_ID_on_keyposegrapher         robot_ID_positions_ID_out_keyposegrapher
    // std::cout << std::endl << "\033[1;32m" << "分类目标点" << "\033[0m" << std::endl;
    for (auto itr : robot_ID_positions_map)  //   分类 目标点
    {
      //  根据位置反算单元   计算  单元里面是否存在   关键位姿图结点，  存在就是可以通行的
      //  对每一个外部       探索点
      std::vector<geometry_msgs::msg::Point> positions_on_keyposegrapher;   //在关键位姿图上
      std::vector<int> positions_ID_on_keyposegrapher;                 //位置索引
      std::vector<geometry_msgs::msg::Point> positions_out_keyposegrapher;  //不在关键位姿图上
      std::vector<int> positions_ID_out_keyposegrapher;                //位置索引

      //  对每一个外部       探索点
      for (int i = 0; i < itr.second.size() - 1; i++)  //在这里减 一     把机器人位置点排除
      {
        int Cell_ID = GetCellInd(itr.second[i].x, itr.second[i].y, itr.second[i].z);  //得到 外部探索点对应的 单元ID
        //  检查  对应点   是否能够连上当前的关键位姿图 上   能连       则放入能连字典   不能连  放入不能连字典。
        bool reachable = false;  //初始定义当前子空间单元不可通行
        if (keypose_graph->IsPositionReachable(itr.second[i]))
        {
          reachable = true;
          positions_on_keyposegrapher.push_back(itr.second[i]);  //加入
          positions_ID_on_keyposegrapher.push_back(i);           //  i  是  robot_ID_positions_map  中
                                                        //  vector<geometry_msgs::msg::Point>  的point  的位置索引
        }
        else  //当不可通行时寻找此单元其他关键位姿结点   存在  就相当于  探索过，能够连接
        {
          // Check all the keypose graph nodes within this cell to see if there are any connected nodes
          // 检查此单元格中的所有keypose图形节点，以查看是否存在任何连接的节点
          double min_dist = DBL_MAX;
          double min_dist_node_ind = -1;
          for (const auto& node_ind : subspaces_->GetCell(Cell_ID).GetGraphNodeIndices())  //相同  单元      存在图结点
          {
            geometry_msgs::msg::Point node_position = keypose_graph->GetNodePosition(node_ind);  //单元内  存在于图上的结点集
            double dist =
                misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(node_position, itr.second[i]);
            if (dist < min_dist)  //找到当前单元图结点上      距离  外部给入探索点  最近的距离
            {
              min_dist = dist;
              min_dist_node_ind = node_ind;
            }
          }
          if (min_dist_node_ind >= 0 && min_dist_node_ind < keypose_graph->GetNodeNum())
          {
            reachable = true;
            // connection_point_geo = keypose_graph->GetNodePosition(min_dist_node_ind);
            positions_on_keyposegrapher.push_back(itr.second[i]);  //加入
            positions_ID_on_keyposegrapher.push_back(i);
          }
        }
        //  可以达到的判定条件  1.在keyposegrapher 中结点 距离在  0.5  内  2. 同一个单元格内存在  结点  认为可以达到
        if (!reachable)  //不可到达，就是不在已探索地方，在外部   且外部是  探索状态 或是u  nsee
        {
          positions_out_keyposegrapher.push_back(itr.second[i]);
          positions_ID_out_keyposegrapher.push_back(i);
        }
      }
      if (positions_on_keyposegrapher.size() > 0)
      {
        robot_ID_positions_on_keyposegrapher.insert(
            std::pair<int, std::vector<geometry_msgs::msg::Point>>(itr.first, positions_on_keyposegrapher));
        robot_ID_positions_ID_on_keyposegrapher.insert(
            std::pair<int, std::vector<int>>(itr.first, positions_ID_on_keyposegrapher));
      }
      if (positions_out_keyposegrapher.size() > 0)
      {
        robot_ID_positions_out_keyposegrapher.insert(
            std::pair<int, std::vector<geometry_msgs::msg::Point>>(itr.first, positions_out_keyposegrapher));
        robot_ID_positions_ID_out_keyposegrapher.insert(
            std::pair<int, std::vector<int>>(itr.first, positions_ID_out_keyposegrapher));
      }
    }

    //打印
    for (auto itr : robot_ID_positions_map)
    {
      std::cout << "\033[1;32m"
                << "机器:" << itr.first << " 外部已探索数量："
                << robot_ID_positions_ID_on_keyposegrapher[itr.first].size() << "\033[0m" << std::endl;
      std::cout << "\033[1;32m"
                << "机器:" << itr.first << " 外部未探索数量："
                << robot_ID_positions_ID_out_keyposegrapher[itr.first].size() << "\033[0m" << std::endl;
      std::cout << "\033[1;32m"
                << "结点CEll_id  :   "
                << "\033[0m" << std::endl;
      for (int i = 0; i < itr.second.size(); i++)
      {
        int Cell_ID = GetCellInd(itr.second[i].x, itr.second[i].y, itr.second[i].z);  //得到 外部探索点对应的 单元ID
        std::cout << "\033[1;32m" << Cell_ID << "\033[0m" << std::endl;
      }
    }

    if (robot_ID_is_fuse.empty())
    {
      std::cout << "\033[1;32m"
                << "融合失败，没有满足条件的子图"
                << "\033[0m" << std::endl;
      return false;
    }

    //    检测   外部已探索点个数  为0   就是没有外部已探索点  融合失败
    for (auto itr : robot_ID_positions_ID_out_keyposegrapher)
    {
      //修改   外部以探索  固定加入起点
      // if(itr.second.empty() || robot_ID_positions_ID_out_keyposegrapher[itr.first].empty())   //若是  外部已探索点集
      // 没有 ,  说明  两个机器人没有共同区域  不融合   或者    外部未探索集没有 说明已探索完毕也不用考虑
      if (robot_ID_positions_ID_out_keyposegrapher[itr.first].empty())
      {
        continue;
        // std::cout << std::endl << "\033[1;32m"<<"不与机器人:" << itr.first<< " 融合"
        // <<robot_ID_positions_ID_on_keyposegrapher[itr.first].size()<< "\033[0m" << std::endl;
      }
      else
      {
        robot_ID_is_fuse[itr.first] = true;  //  连通的且有外部未探索点     就融合成功  这个机器人就得融合进来
      }
    }
    bool is_have_out_explored_point = false;
    for (auto& itr : robot_ID_is_fuse)
    {
      if (itr.second)  //存在外部探索点
      {
        is_have_out_explored_point = true;
        break;
      }
    }
    if (!is_have_out_explored_point)
    {
      std::cout << "\033[1;32m"
                << "没有外部探索点"
                << "\033[0m" << std::endl;
      return false;
    }

    std::cout << "\033[1;32m"
              << "MTSP本地探索点数量： " << exploring_cell_positions.size() << "\033[0m" << std::endl;
    /*
        重写   筛选       参加    MTSP的结点
        查找   探索点   在  MTSP_grid_graph  最近的上的结点号
        查找   机器人位置   在  MTSP_grid_graph  最近的上的结点号
    */
    //选中   外部探索点       及      本地机器人位置    作为     MTSP的  输入
    std::vector<geometry_msgs::msg::Point> exploring_cell_positions_graph;  //融合图   结点位置  数据集
    std::vector<int> exploring_cell_indices_graph;                     //融合图  结点  所在 Cell  单元  编码
    std::vector<int> exploring_graph_node_id;  //融合图  结点  所在   图上的编号   为了求距离
    // std::vector<int> robot_start_id_on_exploring_cell_positions_graph;  //  多机器人  起点
    std::vector<int> robot_start_id_on_MTSP;  //  多机器人  起点   在

    //添加探索点
    //添加外部探索点
    for (auto& itr : robot_ID_positions_out_keyposegrapher)
    {
      //查询外部探索点  在    连通的  MTSP_grid_graph   上  最近的点的   index索引
      for (int i = 0; i < itr.second.size(); i++)
      {
        //在前面分类的时候已经消除 外部机器人点了。
        // if(robot_ID_positions_ID_out_keyposegrapher[itr.first][i]==robot_ID_positions_map[itr.first].size()-1)//排除最后一个点(机器人点)
        // {
        //   continue;
        // }

        geometry_msgs::msg::Point temp_point = itr.second[i];
        int temp_cell_index = GetCellInd(temp_point.x, temp_point.y, temp_point.z);
        //  外部探索点  连接到   机器人   将目标点置为探索点的机器人 位置
        if (subspaces_world_->GetCell(temp_cell_index).GetMTSP_graph_index_find_exploring_cell_() != -1)
        {
          exploring_cell_positions_graph.push_back(temp_point);
          exploring_cell_indices_graph.push_back(temp_cell_index);
          exploring_graph_node_id.push_back(
              subspaces_world_->GetCell(temp_cell_index).GetMTSP_graph_index_find_exploring_cell_());
          std::cout << " 添加外部探索点    "
                    << subspaces_world_->GetCell(temp_cell_index).GetMTSP_graph_index_find_exploring_cell_()
                    << std::endl;
          continue;
        }

        //查询是否存在
        if (MTSP_grid_graph_.find_map_[itr.first].count(temp_cell_index))  //这个需要探索的目标点存在 MTSP_grid_graph 上
        {
          int temp_node_index = MTSP_grid_graph_.find_map_[itr.first][temp_cell_index];
          if (MTSP_grid_graph_.MTSP_graph_nodes_[temp_node_index].is_connected_)
          {
            //添加目标点  到  exploring_cell_positions_graph
            exploring_cell_positions_graph.push_back(temp_point);
            exploring_graph_node_id.push_back(temp_node_index);
            exploring_cell_indices_graph.push_back(temp_cell_index);
            std::cout << " 添加外部探索点    " << temp_node_index << std::endl;
          }
          else
          {
            std::cout << " 外部探索结点 在MTSP_grid_graph上  但  不是连通的" << std::endl;
            //不连通就添加最近的一个点   将    目标点当作   最近的点的编号
            int temp_node_index = -1;
            double temp_node_dist = DBL_MAX;
            std::cout << "MTSP_graph  得到距离  外部探索点  最近的结点" << std::endl;
            MTSP_grid_graph_.GetClosestConnectedNodeIndAndDistance(temp_point, temp_node_index,
                                                                   temp_node_dist);  //得到距离  机器人位置     最近结点
                                                                                     //的关键位姿图上的 结点id 和距离
            if (temp_node_index != -1)
            {
              exploring_cell_positions_graph.push_back(temp_point);
              exploring_graph_node_id.push_back(temp_node_index);
              exploring_cell_indices_graph.push_back(temp_cell_index);
              std::cout << " 添加外部探索点    " << temp_node_index << std::endl;
            }
          }
        }
        else
        {
          std::cout << " 外部探索结点  不在MTSP_grid_graph  上" << std::endl;
          //寻找  外部探索点  在 MTSP_grid_graph 连通点中的    最近点  当作目标点求距离   分为  目标点  和  连通点
          //将目标点和最近点连接
          int temp_node_index = -1;
          double temp_node_dist = DBL_MAX;
          std::cout << "MTSP_graph  得到距离  外部探索点  最近的结点" << std::endl;
          MTSP_grid_graph_.GetClosestConnectedNodeIndAndDistance(temp_point, temp_node_index,
                                                                 temp_node_dist);  //得到距离  机器人位置     最近结点
                                                                                   //的关键位姿图上的 结点id 和距离
          if (temp_node_index != -1)
          {
            exploring_cell_positions_graph.push_back(temp_point);
            exploring_graph_node_id.push_back(temp_node_index);
            exploring_cell_indices_graph.push_back(temp_cell_index);
            std::cout << " 添加外部探索点    " << temp_node_index << std::endl;
          }
        }
      }
    }
    //添加本地探索点
    for (int i = 0; i < exploring_cell_positions.size() - 1; i++)
    {
      geometry_msgs::msg::Point temp_point = exploring_cell_positions[i];
      int temp_cell_index = exploring_cell_indices[i];

      if (subspaces_world_->GetCell(temp_cell_index).GetMTSP_graph_index_find_exploring_cell_() != -1)
      {
        exploring_cell_positions_graph.push_back(temp_point);
        exploring_cell_indices_graph.push_back(temp_cell_index);
        exploring_graph_node_id.push_back(
            subspaces_world_->GetCell(temp_cell_index).GetMTSP_graph_index_find_exploring_cell_());
        std::cout << " 添加本地探索点    "
                  << subspaces_world_->GetCell(temp_cell_index).GetMTSP_graph_index_find_exploring_cell_() << std::endl;
        continue;
      }

      //查询是否存在
      if (MTSP_grid_graph_.find_map_[cur_robot_id_].count(temp_cell_index))  //这个需要探索的目标点存在 MTSP_grid_graph
                                                                             //上
      {
        int temp_node_index = MTSP_grid_graph_.find_map_[cur_robot_id_][temp_cell_index];
        if (MTSP_grid_graph_.MTSP_graph_nodes_[temp_node_index].is_connected_)
        {
          //添加目标点  到  exploring_cell_positions_graph
          exploring_cell_positions_graph.push_back(temp_point);
          exploring_graph_node_id.push_back(temp_node_index);
          exploring_cell_indices_graph.push_back(temp_cell_index);
          std::cout << " 添加本地探索点    " << temp_node_index << std::endl;
        }
        else
        {
          std::cout << " 本地探索结点在MTSP_grid_graph  不是连通的" << std::endl;
          int temp_node_index = -1;
          double temp_node_dist = DBL_MAX;
          std::cout << "MTSP_graph  得到距离  本地探索点  最近的结点" << std::endl;
          MTSP_grid_graph_.GetClosestConnectedNodeIndAndDistance(temp_point, temp_node_index,
                                                                 temp_node_dist);  //得到距离  机器人位置     最近结点
                                                                                   //的关键位姿图上的 结点id 和距离
          if (temp_node_index != -1)
          {
            exploring_cell_positions_graph.push_back(temp_point);
            exploring_graph_node_id.push_back(temp_node_index);
            exploring_cell_indices_graph.push_back(temp_cell_index);
            std::cout << " 添加本地探索点    " << temp_node_index << std::endl;
          }
        }
      }
      else
      {
        std::cout << " 本地探索结点  不在MTSP_grid_graph  上" << std::endl;
        int temp_node_index = -1;
        double temp_node_dist = DBL_MAX;
        std::cout << "MTSP_graph  得到距离  外部探索点  最近的结点" << std::endl;
        MTSP_grid_graph_.GetClosestConnectedNodeIndAndDistance(temp_point, temp_node_index,
                                                               temp_node_dist);  //得到距离  机器人位置     最近结点
                                                                                 //的关键位姿图上的 结点id 和距离
        if (temp_node_index != -1)
        {
          exploring_cell_positions_graph.push_back(temp_point);
          exploring_graph_node_id.push_back(temp_node_index);
          exploring_cell_indices_graph.push_back(temp_cell_index);
          std::cout << " 添加本地探索点    " << temp_node_index << std::endl;
        }
      }
    }
    //添加机器人点
    //  先添加本地机器人点
    int cur_robot_node_index = MTSP_grid_graph_.last_robot_node_index_[cur_robot_id_];
    int cur_robot_cell_index = MTSP_grid_graph_.MTSP_graph_nodes_[cur_robot_node_index].cell_id_;
    geometry_msgs::msg::Point cur_robot_position = MTSP_grid_graph_.MTSP_graph_nodes_[cur_robot_node_index].position_;
    if (MTSP_grid_graph_.MTSP_graph_nodes_[cur_robot_node_index].is_connected_)
    {
      exploring_cell_positions_graph.push_back(cur_robot_position);
      exploring_graph_node_id.push_back(cur_robot_node_index);
      exploring_cell_indices_graph.push_back(cur_robot_cell_index);
      robot_start_id_on_MTSP.push_back(exploring_cell_positions_graph.size() - 1);  //本地机器人放第一个
      std::cout << " 添加本地机器人位置点    " << cur_robot_node_index << std::endl;
    }
    else
    {
      std::cout << " 当前机器人  在MTSP_grid_graph上  但  不是连通的" << std::endl;
      //计算最近的   连通点
      double temp_node_dist = DBL_MAX;
      std::cout << "MTSP_graph  得到距离  外部探索点  最近的结点" << std::endl;
      MTSP_grid_graph_.GetClosestConnectedNodeIndAndDistance(cur_robot_position, cur_robot_node_index,
                                                             temp_node_dist);  //得到距离  机器人位置     最近结点
                                                                               //的关键位姿图上的 结点id 和距离
      if (cur_robot_node_index != -1)
      {
        cur_robot_cell_index = GetCellInd(cur_robot_position.x, cur_robot_position.y, cur_robot_position.z);
        exploring_cell_positions_graph.push_back(cur_robot_position);
        exploring_graph_node_id.push_back(cur_robot_node_index);
        exploring_cell_indices_graph.push_back(cur_robot_cell_index);
        std::cout << " 添加本地机器人位置点    " << cur_robot_node_index << std::endl;
      }
    }
    // 添加外部机器人
    for (auto& itr : MTSP_grid_graph_.last_robot_node_index_)
    {
      if (itr.first == cur_robot_id_)
      {
        continue;
      }
      int ext_robot_node_index = MTSP_grid_graph_.last_robot_node_index_[itr.first];
      int ext_robot_cell_index = MTSP_grid_graph_.MTSP_graph_nodes_[ext_robot_node_index].cell_id_;
      geometry_msgs::msg::Point ext_robot_position = MTSP_grid_graph_.MTSP_graph_nodes_[ext_robot_node_index].position_;
      if (MTSP_grid_graph_.MTSP_graph_nodes_[ext_robot_node_index].is_connected_)
      {
        exploring_cell_positions_graph.push_back(ext_robot_position);
        exploring_graph_node_id.push_back(ext_robot_node_index);
        exploring_cell_indices_graph.push_back(ext_robot_cell_index);
        robot_start_id_on_MTSP.push_back(exploring_cell_positions_graph.size() - 1);
        std::cout << " 添加外部机器人位置点    " << ext_robot_node_index << std::endl;
      }
      else
      {
        std::cout << " 外部机器人  在MTSP_grid_graph上  但  不是连通的" << std::endl;
        double temp_node_dist = DBL_MAX;
        std::cout << "MTSP_graph  得到距离  外部探索点  最近的结点" << std::endl;
        MTSP_grid_graph_.GetClosestConnectedNodeIndAndDistance(ext_robot_position, ext_robot_node_index,
                                                               temp_node_dist);  //得到距离  机器人位置     最近结点
                                                                                 //的关键位姿图上的 结点id 和距离
        if (ext_robot_node_index != -1)
        {
          ext_robot_cell_index = GetCellInd(ext_robot_position.x, ext_robot_position.y, ext_robot_position.z);
          exploring_cell_positions_graph.push_back(ext_robot_position);
          exploring_graph_node_id.push_back(ext_robot_node_index);
          exploring_cell_indices_graph.push_back(ext_robot_cell_index);
          std::cout << " 添加外部机器人位置点    " << ext_robot_node_index << std::endl;
        }
      }
    }
    //计算距离矩阵
    int MTSP_node_num = exploring_cell_positions_graph.size();
    std::vector<std::vector<int>> distance_matrix_graph(MTSP_node_num,
                                                        std::vector<int>(MTSP_node_num, 0));  //融合图的距离矩阵
    //构建距离矩阵
    for (int i = 0; i < MTSP_node_num; i++)
    {
      for (int j = 0; j < i; j++)
      {
        if (!use_keypose_graph_ || keypose_graph == nullptr ||
            keypose_graph->GetNodeNum() == 0)  //不使用关键位姿图   直接用两点 直线距离构造边
        {
          // Use straight line connection     初始化  使用直线连接
          distance_matrix_graph[i][j] =
              static_cast<int>(10 * misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
                                        exploring_cell_positions_graph[i], exploring_cell_positions_graph[j]));
        }
        else
        {
          // Use  MTSP_grid_graph
          nav_msgs::msg::Path path_tmp;
          distance_matrix_graph[i][j] = static_cast<int>(
              10 * GetShortestPath_MTSP_grid_graph(exploring_graph_node_id[i],
                                                   exploring_graph_node_id[j]));  //在 关键位姿图上   找到最短距离
          std::cout << "融合图上   " << exploring_graph_node_id[i] << "     和   " << exploring_graph_node_id[j]
                    << "    路径长度  " << distance_matrix_graph[i][j] << std::endl;
        }
      }
    }
    //对角化
    for (int i = 0; i < MTSP_node_num; i++)
    {
      for (int j = i + 1; j < MTSP_node_num; j++)
      {
        distance_matrix_graph[i][j] = distance_matrix_graph[j][i];
      }
    }
    std::cout << std::endl
              << "\033[1;32m"
              << "成功"
              << "\033[0m" << std::endl;
    //赋值   传出
    exploring_cell_indices_MTSP = exploring_cell_indices_graph;
    std::cout << "\033[1;32m"
              << "融合图结点单元 Cell _ID: "
              << "\033[0m" << std::endl;
    for (int j = 0; j < exploring_cell_indices_graph.size(); j++)
    {
      std::cout << "\033[1;32m" << exploring_cell_indices_graph[j] << "\033[0m" << std::endl;
    }
    exploring_cell_positions_MTSP = exploring_cell_positions_graph;
    distance_matrix_MTSP = distance_matrix_graph;
    // robot_position_id_on_exploring_cell_positions_MTSP.push_back(robot_ip);   //   本地机器人放第一个
    positions_MTSP_grid_graph_node_id = exploring_graph_node_id;
    for (int i = 0; i < robot_start_id_on_MTSP.size(); i++)
    {
      robot_position_id_on_exploring_cell_positions_MTSP.push_back(robot_start_id_on_MTSP[i]);
    }
    return true;
  }
  else  //没有外部输入
  {
    std::cout << "没有外部传入" << std::endl;
    return false;
  }
}

//查询  MTSP_grid_graph  两个连通结点的最小距离
double GridWorld::GetShortestPath_MTSP_grid_graph(int start_id, int target_id)
{
  if (MTSP_grid_graph_.MTSP_graph_nodes_.size() < 2)
  {
    return misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
        MTSP_grid_graph_.MTSP_graph_nodes_[start_id].position_,
        MTSP_grid_graph_.MTSP_graph_nodes_[target_id].position_);
  }
  int from_idx = start_id;
  int to_idx = target_id;
  double min_dist_to_start = DBL_MAX;
  double min_dist_to_target = DBL_MAX;
  std::vector<geometry_msgs::msg::Point> node_positions;
  for (int i = 0; i < MTSP_grid_graph_.MTSP_graph_nodes_.size(); i++)
  {
    node_positions.push_back(MTSP_grid_graph_.MTSP_graph_nodes_[i].position_);
  }
  bool get_path = false;  //不要路径。
  std::vector<int> path_indices;
  double shortest_dist = misc_utils_ns::AStarSearch(MTSP_grid_graph_.graph_, MTSP_grid_graph_.dist_, node_positions,
                                                    from_idx, to_idx, get_path, path_indices);
  return shortest_dist;
}

//返回 MTSP_grid_graph  两个连通结点的路经索引
std::vector<int> GridWorld::GetShortestPath_MTSP_grid_graph_path(int start_id, int target_id)
{
  std::vector<int> path_indices;
  if (MTSP_grid_graph_.MTSP_graph_nodes_.size() < 2)
  {
    std::cout << "融合图点个数" << std::endl;
    return path_indices;  //返回空
  }
  int from_idx = start_id;
  int to_idx = target_id;
  double min_dist_to_start = DBL_MAX;
  double min_dist_to_target = DBL_MAX;
  std::vector<geometry_msgs::msg::Point> node_positions;
  for (int i = 0; i < MTSP_grid_graph_.MTSP_graph_nodes_.size(); i++)
  {
    node_positions.push_back(MTSP_grid_graph_.MTSP_graph_nodes_[i].position_);
  }
  double shortest_dist = misc_utils_ns::AStarSearch(MTSP_grid_graph_.graph_, MTSP_grid_graph_.dist_, node_positions,
                                                    from_idx, to_idx, true, path_indices);
  std::cout << "MTSP_grid_graph:     index  " << from_idx << " 和 "
            << "to_idx   "
            << " 长度   " << shortest_dist << " 路径长度  " << path_indices.size() << std::endl;
  std::cout << "路经点： " << std::endl;
  for (int i = 0; i < path_indices.size(); i++)
  {
    std::cout << path_indices[i] << "  ->";
  }
  std::cout << std::endl;
  return path_indices;
}

//添加函数
int GridWorld::GetNavigationIndexFromMtspGridGraph(
    std::vector<int>& path_node_index, const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
    std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph, bool& use_Navigation_node)
{
  //查询选用  哪个点做导航点
  //目标点:
  if (path_node_index.empty())
  {
    return -1;
  }

  //优先级最高   查询  从后向前     第一个  在   keypose_graph  上能够到达的点  当作导航点
  std::cout << "计算导航点   在  keypose_graph  " << std::endl;
  int keypose_graph_node_index = -1;
  for (int i = path_node_index.size() - 1; i >= 0; i--)
  {
    int temp_index = path_node_index[i];
    geometry_msgs::msg::Point temp_position = MTSP_grid_graph_.MTSP_graph_nodes_[temp_index].position_;
    Eigen::Vector3d temp_check_position = Eigen::Vector3d(temp_position.x, temp_position.y, temp_position.z);
    //  查询点
    int temp = -1;
    double dis = DBL_MAX;
    keypose_graph->GetClosestConnectedNodeIndAndDistance(temp_position, temp, dis);
    if (temp != -1 && dis <= 0.5 * kCellSize)
    {
      // if(i==path_node_index.size()-1) //最后的一个点在  探测 的范围内，直接使用
      // {
      //   use_Navigation_node=false;
      //   return  -1;
      // }
      keypose_graph_node_index = temp_index;
      break;
    }
  }
  std::cout << "计算导航点   在  keypose_graph  " << std::endl;

  int goal_index = path_node_index.back();
  geometry_msgs::msg::Point goal_position = MTSP_grid_graph_.MTSP_graph_nodes_[goal_index].position_;
  Eigen::Vector3d goal_check_position = Eigen::Vector3d(goal_position.x, goal_position.y, goal_position.z);

  //查询从此之后的所有路经点   是否在  都在  局部规划里面
  bool is_local_planner_range = true;
  //记录第一个不在局部规划框的点
  int first_out_local_planning_index = -1;
  for (int i = 0; i < path_node_index.size(); i++)
  {
    int temp_index = path_node_index[i];
    geometry_msgs::msg::Point temp_position = MTSP_grid_graph_.MTSP_graph_nodes_[temp_index].position_;
    Eigen::Vector3d temp_check_position = Eigen::Vector3d(temp_position.x, temp_position.y, temp_position.z);
    int viewpoint_ind = viewpoint_manager->GetViewPointInd(temp_check_position);  //得到  点 的  局部规划id
    if (!viewpoint_manager->InRange(viewpoint_ind))                               //不在规划框内
    {
      //只要其中有一个不在局部规划框内  就    false    跳出
      is_local_planner_range = false;
      first_out_local_planning_index = temp_index;  //得到最后一个不在局部规划框内的点
      break;
    }
  }

  if (!is_local_planner_range)  //不在局部规划框内  本地信息边界点当导航点
  {
    //  获得候选点   在本地探索点    信息边界的候选点
    std::vector<int> canditate_nodes;
    std::vector<int> canditate_nodes_path;
    for (int i = 0; i < path_node_index.size(); i++)
    {
      //在本地局部信息的边界  及（本地局部状态为探索的单元格里面的点）
      int temp_index = path_node_index[i];
      geometry_msgs::msg::Point temp_position = MTSP_grid_graph_.MTSP_graph_nodes_[temp_index].position_;
      int temp_cell_index = MTSP_grid_graph_.MTSP_graph_nodes_[temp_index].cell_id_;
      if (subspaces_local_->GetCell(temp_cell_index).GetStatus() == CellStatus::EXPLORING)  //本地信息边界
      {
        canditate_nodes.push_back(temp_index);
        canditate_nodes_path.push_back(i);
      }
    }

    if (canditate_nodes.empty())  //选在局部规划框之外的第一个点当导航点
    {
      if (keypose_graph_node_index != -1)
      {
        return keypose_graph_node_index;
      }
      if (first_out_local_planning_index != -1)
      {
        return first_out_local_planning_index;
      }
    }
    //不为空时  继续  优先级为   keypose 点
    int Navigation_Index = -1;
    int index_path = 0;
    for (int i = 0; i < canditate_nodes.size();
         i++)  //从前到后   一直更新，为最远的一个探索状态  中的  是  keypose 点(机器人路经点)
    {
      if (MTSP_grid_graph_.MTSP_graph_nodes_[canditate_nodes[i]].is_keypose_)
      {
        Navigation_Index = canditate_nodes[i];
        index_path = canditate_nodes_path[i];
      }
    }
    if (Navigation_Index != -1 && index_path != 0)
    {
      //最远的点一个  keypose  一定要够远
      double length;
      nav_msgs::msg::Path temp_path;
      for (int i = 0; i <= index_path; i++)
      {
        //计算路经长度  一定大于部分长度
        int temp_index = path_node_index[i];
        geometry_msgs::msg::Point temp_position = MTSP_grid_graph_.MTSP_graph_nodes_[temp_index].position_;
        geometry_msgs::msg::PoseStamped temp_pose;
        temp_pose.pose.position = temp_position;
        temp_path.poses.push_back(temp_pose);
      }
      length = misc_utils_ns::GetPathLength(temp_path);
      if (length > 2 * kCellSize)  //路经大于两个单元格
      {
        return Navigation_Index;
      }
    }
    //当在这里面没有找到  keypose点
    //查询  canditate_nodes.back()  的距离是否满足较远距离
    double length;
    nav_msgs::msg::Path temp_path;
    for (int i = 0; i <= canditate_nodes_path.back(); i++)
    {
      //计算路经长度  一定大于部分长度
      int temp_index = path_node_index[i];
      geometry_msgs::msg::Point temp_position = MTSP_grid_graph_.MTSP_graph_nodes_[temp_index].position_;
      geometry_msgs::msg::PoseStamped temp_pose;
      temp_pose.pose.position = temp_position;
      temp_path.poses.push_back(temp_pose);
    }
    length = misc_utils_ns::GetPathLength(temp_path);
    if (length > 2 * kCellSize)  //路经大于两个单元格
    {
      return canditate_nodes.back();
    }

    if (keypose_graph_node_index != -1)
    {
      return keypose_graph_node_index;
    }
    if (first_out_local_planning_index != -1)
    {
      return first_out_local_planning_index;
    }
  }
  else  //在局部规划框内选   目标点当导航点
  {
    return path_node_index.back();
  }

  return path_node_index.back();
}

//构造函数
MTSP_grid_graph::MTSP_grid_graph()
{
  kdtree_connected_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  connected_nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  kdtree_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
}

bool MTSP_grid_graph::GetCloseConnectNodeMtspGraph(geometry_msgs::msg::Point& robot_pose, int& graph_index, double& dist,
                                                   double& addthr)
{
  if (connected_nodes_cloud_->points.empty())
  {
    graph_index = -1;
    dist = DBL_MAX;
    return false;
  }
  pcl::PointXYZI search_point;
  search_point.x = robot_pose.x;
  search_point.y = robot_pose.y;
  search_point.z = robot_pose.z;
  std::vector<int> nearest_neighbor_node_indices(1);
  std::vector<float> nearest_neighbor_squared_dist(1);
  kdtree_connected_nodes_->nearestKSearch(
      search_point, 1, nearest_neighbor_node_indices,
      nearest_neighbor_squared_dist);  //  以 search_point  为基准点，取 1 个点  ，最近点的
                                       //  nearest_neighbor_node_indices 下标，最近点的距离nearest_neighbor_squared_dist
  if (!nearest_neighbor_node_indices.empty() && nearest_neighbor_node_indices.front() >= 0 &&
      nearest_neighbor_node_indices.front() < connected_nodes_cloud_->points.size())
  {
    graph_index = static_cast<int>(connected_nodes_cloud_->points[nearest_neighbor_node_indices.front()].intensity);
    dist = sqrt(nearest_neighbor_squared_dist.front());
  }
  else
  {
    // RCLCPP_WARN_STREAM(this->get_logger(), "search for nearest neighbor failed with " << connected_nodes_cloud_->points.size());
    graph_index = -1;
    dist = -1;
  }
  if (dist >= 0 && dist < addthr)  //存在  阈值范围内的  连接点  不用添加
  {
    return true;
  }
  return false;
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
            keypose_node.keypose_graph_node_ind_ = static_cast<int>(cur_keypose_path.poses[i].pose.orientation.x);
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
 *                                              使用  merger graph  做全局规划  替代  keypose graph
 *                                                               SolveGlobalMdvrp_merger_graph
 *
 * ************************************************************************************/
exploration_path_ns::ExplorationPath GridWorld::SolveGlobalMdvrp_merger_graph(
    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
    std::vector<int>& ordered_cell_indices, bool& is_global_tsp,
    std::vector<rclcpp::Client<tare_planner::srv::RequestPath>::SharedPtr>& request_path_client_list_, // 替换为ROS2
    std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
    std::shared_ptr<merger_graph_ns::MergerGraph>& merger_graph,
    std::map<int, geometry_msgs::msg::Point>& other_robot_position_map)
{
  const bool debug = true;
  // 中心思想  使用  merger  graph  代替 keypsoe graph
  /*
   *  流程：
      判定  机器人状态  前一时刻为导航状态
              使用  目标网格  求  路径即可 再传出
              若前一时刻 为探索状态  且  不存在局部路径
              做全局搜索  Mdvrp  分配  获得目标网格
              求路径再传出
   *
   */
  /*******************状态判定****************************/

  //    在这三个状态下，只要局部规划有了目标点  则优先探索目标点
  exploration_path_ns::ExplorationPath global_path;  // 最终发出的路径
  // 计算  机器人位置 和  索引
  /*********获取与机器人位置相关联的keypose图上的节点* *****/
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
      // std::cout << std::endl
      //           << "\033[1;32m"
      //           << "切换状态，从far导航  变为探索状态，执行局部规划"
      //           << "\033[0m" << std::endl;
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

  if (debug)
  {
    std::cout << "[GW][MTSP] allocation_strategy=" << allocation_strategy_
              << " other_robots=" << other_robot_position_map.size()
              << " neighbor_exclude=" << neighbor_cell_indices_all.size()
              << " exploring_world=" << exploring_cell_indices_world.size() << std::endl;
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
  if (debug)
  {
    std::cout << "[GW][MTSP] depot order (robot_id sorted): ";
    for (int i = 0; i < robot_ids_sorted.size(); i++)
    {
      if (i != 0)
      {
        std::cout << ",";
      }
      std::cout << robot_ids_sorted[i];
    }
    std::cout << std::endl;
  }
  // std::cout << "参与探索机器人个数   =  " << robot_position_id_on_exploring_cell_positions_world.size() << std::endl;

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
        // Use keypose graph   使用关键位姿图
        nav_msgs::msg::Path path_tmp;
        distance_matrix_world[i][j] = static_cast<int>(
            10 * merger_graph->GetShortestPath(exploring_cell_positions_world[i], exploring_cell_positions_world[j],
                                               false, path_tmp, true));  //在 关键位姿图上   找到最短距离       true
                                                                         //使用  连接点  考虑  高度约束
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

  //  有了  机器人    位置  和   距离矩阵

  // is_fuse_  用来标记     机器人 多就做融合    是做Mdvrp 还是  单车 TSP
  if (robot_position_id_on_exploring_cell_positions_world.size() > 1)
  {
    is_fuse_ = true;
  }

  //  返回起点
  return_home_ = false;
  //  需要全局  规划    但全局没有  可探索点
  // 全局返回使用  merger graph
  if (is_global_tsp == true && num_world_neighbor_cell_reachable == 0)
  {
    // return_home_ = true;
    // std::cout << "return_home" << std::endl;

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
  // std::cout << "return_home_    " << return_home_ << std::endl;

  std::vector<int> node_index;
  if (!is_fuse_)  //融合失败  做      TSP    用全局可到达      exploring_cell_positions_world
  {
    /****** Solve the TSP  求解   旅行商(TSP)问题 优化 ******/
    tsp_solver_ns::DataModel data_model;                           //数据模型
    data_model.distance_matrix = distance_matrix_world;            //距离矩阵
    data_model.depot = exploring_cell_positions_world.size() - 1;  //起点  就是  最后的一个
    tsp_solver_ns::TSPSolver tsp_solver(data_model);
    tsp_solver.Solve();
    // std::cout << "打印TSP结果" << std::endl;
    tsp_solver.getSolutionNodeIndex(node_index, false);
    // std::cout << "  获取结果" << std::endl;
    //更新 变量方便后续处理
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

    // // 验证  Mdvrp是否  成功
    // bool debug = false;
    // if (debug)
    // {
    //   std::cout << "---------------Mdvrp验证------------------" << std::endl;
    //   std::cout << "  节点个数            :   " << distance_matrix_world.size() << std::endl;
    //   std::cout << "  机器人个数        :   " << robot_num << std::endl;
    //   for (int j = 0; j < robot_num; j++)
    //   {
    //     std::cout << "  机器人索引号   :   " << robot_position_id_on_exploring_cell_positions_world[j] << std::endl;
    //   }
    //   std::cout << "  邻接矩阵        :   " << robot_num << std::endl;
    //   // 邻接矩阵 exploring_cell_indices_world
    //   for (int i = 0; i < distance_matrix_world.size(); i++)
    //   {
    //     for (int j = 0; j < distance_matrix_world.size(); j++)
    //     {
    //       std::cout << distance_matrix_world[i][j] << "  ";
    //     }
    //     std::cout << std::endl;
    //   }
    //   // 分配结果
    //   std::cout << "  分配结果        :   " << robot_num << std::endl;
    //   for (auto& x : node_index)
    //   {
    //     std::cout << x << " ---> ";
    //   }
    //   std::cout << std::endl;
    // }

    // //打印输出  查看
    // std::cout << std::endl
    //           << "\033[1;32m"
    //           << "做 Mdvrp"
    //           << "\033[0m" << std::endl;
  }

  if (debug)
  {
    std::cout << "[GW][MTSP] node_index.size=" << node_index.size() << " mapped_cell_ids=";
    for (int i = 0; i < node_index.size(); i++)
    {
      int idx = node_index[i];
      if (idx >= 0 && idx < static_cast<int>(exploring_cell_indices.size()))
      {
        std::cout << exploring_cell_indices[idx] << " ";
      }
      else
      {
        std::cout << "(idx:" << idx << ") ";
      }
    }
    std::cout << std::endl;
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
    bool debug = false;
    if (debug)
    {
      std::cout << std::endl
                << "\033[1;32m"
                << "robot_cell_id:          " << robot_cell_id << "\033[0m" << std::endl;
      //打印  需要探索的 序号
      std::cout << "\033[1;32m"
                << "exploring_cell_indices:          "
                << "\033[0m" << std::endl;
      for (int i = 0; i < exploring_cell_indices.size(); i++)
      {
        std::cout << "\033[1;32m" << exploring_cell_indices[i] << "     "
                  << "\033[0m";
      }
      //  打印查看   全局路径  Cell  id
      for (int i = 0; i < node_index.size(); i++)
      {
        if (i == 0)
        {
          std::cout << std::endl
                    << "\033[1;32m"
                    << "排序  结点数量：" << node_index.size() << "\033[0m" << std::endl;
          std::cout << "\033[1;32m" << exploring_cell_indices[node_index[i]] << " ---> "
                    << "\033[0m";
        }
        else if (i == node_index.size() - 1)
        {
          std::cout << "\033[1;32m" << exploring_cell_indices[node_index[i]] << "\033[0m";
        }
        else
        {
          std::cout << "\033[1;32m" << exploring_cell_indices[node_index[i]] << " ---> "
                    << "\033[0m";
        }
      }
    }
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
      // std::cout << "   做Mdvrp   " << std::endl;
      int robot_num = robot_position_id_on_exploring_cell_positions_world.size();  //机器人数量
      int exploring_num =
          distance_matrix_world.size() - robot_position_id_on_exploring_cell_positions_world.size();  //外部未探索点数量
      exploration_path_ns::Node node_goal;
      int goal_ind;
      if (node_index.size() <= 2 && exploring_num > 0)  // mdvrp       没有给本地机器人分配目标点
                                                        // 但还是存在可达到的未探索位置     修正  探索点
                                                        // 给入（贪婪，最近的一个未探索点当作目标点）
      {
        // std::cout << "   mdvrp   没有分配  探索补偿目标点   " << std::endl;
        //选取   距离当前机器人最近的  未探索   目标点    给入
        std::vector<int> distance_vector =
            distance_matrix_world[robot_position_id_on_exploring_cell_positions_world.front()];
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

        int minPosition = std::min_element(goal_position_distance.begin(), goal_position_distance.end()) -
                          goal_position_distance.begin();  //得到最小点的索引
        //加入本地

        //找到    目标点
        exploration_path_ns::Node node_goal_(Eigen::Vector3d(goal_position[minPosition].x, goal_position[minPosition].y,
                                                             goal_position[minPosition].z));  //当前位置
        node_goal_.type_ = exploration_path_ns::NodeType::GLOBAL_VIEWPOINT;
        node_goal_.global_subspace_index_ =
            GetCellInd(goal_position[minPosition].x, goal_position[minPosition].y, goal_position[minPosition].z);
        node_goal = node_goal_;
        goal_ind = goal_MTSP_index[minPosition];  // goal_ind  在
      }
      else if (node_index.size() <= 2 && exploring_num == 0)  // mtsp  没有目标点  则  return home
                                                              // ?还是跟踪其中一个机器人
      {
        // std::cout << "   Mdvrp   没有目标点     " << std::endl;
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
        // std::cout << "   Mdvrp   正常分配目标点     " << std::endl;
        //本地正常有结果

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

      //运行到这   说明     是从  Mdvrp  计算  而来   也可能是   修正过的
      //当前机器人位置
      int robot_ind = node_index[0];
      geometry_msgs::msg::Point cur_position;
      cur_position = exploring_cell_positions[robot_ind];
      // std::cout << "cur_position    " << cur_position << std::endl;
      exploration_path_ns::Node node_local(Eigen::Vector3d(exploring_cell_positions[robot_ind].x,
                                                           exploring_cell_positions[robot_ind].y,
                                                           exploring_cell_positions[robot_ind].z));  //当前位置
      node_local.type_ = exploration_path_ns::NodeType::ROBOT;
      node_local.global_subspace_index_ =
          GetCellInd(exploring_cell_positions[robot_ind].x, exploring_cell_positions[robot_ind].y,
                     exploring_cell_positions[robot_ind].z);
      global_path.Append(node_local);

      // std::cout << "采取   merger   graph  导航   far方式" << std::endl;
      //单线的形式    目标点  node_goal
      // 求    node_local ------> node_goal_  在  merger graph  上的  路径
      nav_msgs::msg::Path merger_graph_path;  //路径
      merger_graph->GetShortestPath(cur_position, exploring_cell_positions[goal_ind], true, merger_graph_path,
                                    true);  //得到两个位置的   连接路径
      ordered_cell_indices.push_back(exploring_cell_indices[robot_ind]);
      ordered_cell_indices.push_back(exploring_cell_indices[goal_ind]);

      // Fill in the path in between        填充中间的路径   不存在路径  就无所谓
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

      //可能前一个位置 和  下一个位置  相差远，一个在位姿关键图上  ，一个不在位姿关键图上   ，需要插值
      //  判定   在位姿关键图上   按以前的思路插值。
      //判定   当前位置 和  下一个位置  是否在关键位置图上

      nav_msgs::msg::Path keypose_path;  //路径
      // keypose_graph->GetShortestPath(cur_position, next_position, true, keypose_path, false);  //得到两个位置的
      // 连接路径       false
      keypose_graph->GetShortestPath(cur_position, next_position, true, keypose_path,
                                     true);  //得到两个位置的   连接路径       false
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