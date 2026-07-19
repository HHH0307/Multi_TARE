//
// Created by caochao on 12/31/19.
//
#include "rcutils/error_handling.h"
#include <keypose_graph/keypose_graph.h>
#include <viewpoint_manager/viewpoint_manager.h>
#include <stack>

namespace keypose_graph_ns
{
KeyposeNode::KeyposeNode(double x, double y, double z, int node_ind, int keypose_id, bool is_keypose)
  : cell_ind_(0), node_ind_(node_ind), keypose_id_(keypose_id), is_keypose_(is_keypose), is_connected_(true)
{
  position_.x = x;
  position_.y = y;
  position_.z = z;

  offset_to_keypose_.x = 0.0;
  offset_to_keypose_.y = 0.0;
  offset_to_keypose_.z = 0.0;
}

KeyposeNode::KeyposeNode(const geometry_msgs::msg::Point& point, int node_ind, int keypose_id, bool is_keypose)
  : KeyposeNode(point.x, point.y, point.z, node_ind, keypose_id, is_keypose)
{
}

//默认的  构造函数
KeyposeGraph::KeyposeGraph()
  : allow_vertical_edge_(false)
  , current_keypose_id_(0)
  , kAddNodeMinDist(1.0)
  , kAddEdgeCollisionCheckResolution(0.4)
  , kAddEdgeCollisionCheckRadius(0.3)
  , kAddEdgeConnectDistThr(3.0)
  , kAddEdgeToLastKeyposeDistThr(3.0)
  , kAddEdgeVerticalThreshold(1.0)
  , kAddEdgeCollisionCheckPointNumThr(1)
{
  kdtree_connected_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  connected_nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  kdtree_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  // 初始化
  new_keypose_ind_ = 0;

  // 新增  共享 初始化  给robot id
  Shared_Infor_.robot_id = 0;  // 初始化为  0   在发送的时候给值。
}

KeyposeGraph::KeyposeGraph(rclcpp::Node::SharedPtr nh)
  : allow_vertical_edge_(false)
  , current_keypose_id_(0)
  , kAddNodeMinDist(1.0)
  , kAddEdgeCollisionCheckResolution(0.4)
  , kAddEdgeCollisionCheckRadius(0.3)
  , kAddEdgeConnectDistThr(3.0)
  , kAddEdgeToLastKeyposeDistThr(3.0)
  , kAddEdgeVerticalThreshold(1.0)
  , kAddEdgeCollisionCheckPointNumThr(1)
{
  ReadParameters(nh);
  kdtree_connected_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  connected_nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  kdtree_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  // 初始化
  new_keypose_ind_ = 0;

  // 新增  共享 初始化  给robot id
  Shared_Infor_.robot_id = 0;  // 初始化为  0   在发送的时候给值。
}

void KeyposeGraph::ReadParameters(rclcpp::Node::SharedPtr nh)
{
  nh->get_parameter("keypose_graph/kAddNodeMinDist", kAddNodeMinDist);
  nh->get_parameter("keypose_graph/kAddNonKeyposeNodeMinDist", kAddNonKeyposeNodeMinDist);
  nh->get_parameter("keypose_graph/kAddEdgeConnectDistThr", kAddEdgeConnectDistThr);
  nh->get_parameter("keypose_graph/kAddEdgeToLastKeyposeDistThr", kAddEdgeToLastKeyposeDistThr);
  nh->get_parameter("keypose_graph/kAddEdgeVerticalThreshold", kAddEdgeVerticalThreshold);
  nh->get_parameter("keypose_graph/kAddEdgeCollisionCheckResolution", kAddEdgeCollisionCheckResolution);
  nh->get_parameter("keypose_graph/kAddEdgeCollisionCheckRadius", kAddEdgeCollisionCheckRadius);
  nh->get_parameter("keypose_graph/kAddEdgeCollisionCheckPointNumThr", kAddEdgeCollisionCheckPointNumThr);
  int Local_planning_x_num = nh->get_parameter("viewpoint_manager/number_x").as_int(); // 新增 + 修改 2025.10.28
  double Local_planning_resolution_x = nh->get_parameter("viewpoint_manager/resolution_x").as_double(); // 新增 + 修改 2025.10.28
  kLocal_planning_radius = Local_planning_x_num * Local_planning_resolution_x * 0.5;  // 15m
}

void KeyposeGraph::AddNode(const geometry_msgs::msg::Point& position, int node_ind, int keypose_id, bool is_keypose)
{
  KeyposeNode new_node(position, node_ind, keypose_id, is_keypose);
  nodes_.push_back(new_node);
  std::vector<int> neighbors;
  graph_.push_back(neighbors);
  std::vector<double> neighbor_dist;
  dist_.push_back(neighbor_dist);

  // 添加节点信息到共享信息
  tare_planner::msg::SharedNode new_shared_node;
  new_shared_node.node_index = node_ind;
  new_shared_node.node_position = position;
  new_shared_node.is_keypose = is_keypose;
  Shared_Infor_.new_node_set.push_back(new_shared_node);
}

// 重载添加属性
void KeyposeGraph::AddNode(const geometry_msgs::msg::Point& position, int node_ind, int keypose_id, bool is_keypose, int robot_id)
{
    KeyposeNode new_node(position, node_ind, keypose_id, is_keypose);
    new_node.robot_id_ = robot_id;  // 添加节点所属机器人的属性
    nodes_.push_back(new_node);
    std::vector<int> neighbors;
    graph_.push_back(neighbors);
    std::vector<double> neighbor_dist;
    dist_.push_back(neighbor_dist);

    // 添加节点信息到共享信息
    tare_planner::msg::SharedNode new_shared_node;
    new_shared_node.node_index = node_ind;
    new_shared_node.node_position = position;
    new_shared_node.is_keypose = is_keypose;
    Shared_Infor_.new_node_set.push_back(new_shared_node);
}

void KeyposeGraph::AddNodeAndEdge(const geometry_msgs::msg::Point& position, int node_ind, int keypose_id,
                                  bool is_keypose, int connected_node_ind, double connected_node_dist)
{
  AddNode(position, node_ind, keypose_id, is_keypose);
  AddEdge(connected_node_ind, node_ind, connected_node_dist);
}

void KeyposeGraph::AddEdge(int from_node_ind, int to_node_ind, double dist)
{
  MY_ASSERT(from_node_ind >= 0 && from_node_ind < graph_.size() && from_node_ind < dist_.size());
  MY_ASSERT(to_node_ind >= 0 && to_node_ind < graph_.size() && to_node_ind < dist_.size());

  graph_[from_node_ind].push_back(to_node_ind);
  graph_[to_node_ind].push_back(from_node_ind);

  dist_[from_node_ind].push_back(dist);
  dist_[to_node_ind].push_back(dist);

  // 添加边  信息
  tare_planner::msg::SharedEdge new_edge;
  if (from_node_ind > to_node_ind)
  {
    new_edge.node_index_min = to_node_ind;
    new_edge.node_index_max = from_node_ind;
  }
  else
  {
    new_edge.node_index_min = from_node_ind;
    new_edge.node_index_max = to_node_ind;
  }
  Shared_Infor_.add_edge_set.push_back(new_edge);
}

bool KeyposeGraph::HasNode(const Eigen::Vector3d& position)
{
  int closest_node_ind = -1;
  double min_dist = DBL_MAX;
  geometry_msgs::msg::Point geo_position;
  geo_position.x = position.x();
  geo_position.y = position.y();
  geo_position.z = position.z();
  GetClosestNodeIndAndDistance(geo_position, closest_node_ind, min_dist);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
  {
    double xy_dist = misc_utils_ns::PointXYDist<geometry_msgs::msg::Point>(geo_position, nodes_[closest_node_ind].position_);
    double z_dist = std::abs(geo_position.z - nodes_[closest_node_ind].position_.z);
    if (xy_dist < kAddNonKeyposeNodeMinDist && z_dist < 1.0)
    {
      return true;
    }
  }
  return false;
}

bool KeyposeGraph::HasEdgeBetween(int node_ind1, int node_ind2)
{
  if (node_ind1 >= 0 && node_ind1 < nodes_.size() && node_ind2 >= 0 && node_ind2 < nodes_.size())
  {
    if (std::find(graph_[node_ind1].begin(), graph_[node_ind1].end(), node_ind2) != graph_[node_ind1].end() ||
        std::find(graph_[node_ind2].begin(), graph_[node_ind2].end(), node_ind1) != graph_[node_ind2].end())
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool KeyposeGraph::IsConnected(const Eigen::Vector3d& from_position, const Eigen::Vector3d& to_position)
{
  geometry_msgs::msg::Point from_node_position;
  from_node_position.x = from_position.x();
  from_node_position.y = from_position.y();
  from_node_position.z = from_position.z();
  int closest_from_node_ind = -1;
  double closest_from_node_dist = DBL_MAX;
  GetClosestNodeIndAndDistance(from_node_position, closest_from_node_ind, closest_from_node_dist);

  geometry_msgs::msg::Point to_node_position;
  to_node_position.x = to_position.x();
  to_node_position.y = to_position.y();
  to_node_position.z = to_position.z();
  int closest_to_node_ind = -1;
  double closest_to_node_dist = DBL_MAX;
  GetClosestNodeIndAndDistance(to_node_position, closest_to_node_ind, closest_to_node_dist);

  if (closest_from_node_ind != -1 && closest_from_node_ind == closest_to_node_ind)
  {
    return true;
  }
  else if (HasEdgeBetween(closest_from_node_ind, closest_to_node_ind))
  {
    return true;
  }
  else
  {
    return false;
  }
}

int KeyposeGraph::AddNonKeyposeNode(const geometry_msgs::msg::Point& new_node_position)
{
  int new_node_index = -1;
  int closest_node_ind = -1;
  double closest_node_dist = DBL_MAX;
  GetClosestNodeIndAndDistance(new_node_position, closest_node_ind, closest_node_dist);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
  {
    double xy_dist =
        misc_utils_ns::PointXYDist<geometry_msgs::msg::Point>(new_node_position, nodes_[closest_node_ind].position_);
    double z_dist = std::abs(new_node_position.z - nodes_[closest_node_ind].position_.z);
    if (xy_dist < kAddNonKeyposeNodeMinDist && z_dist < 1.0)
    {
      return closest_node_ind;
    }
  }
  new_node_index = nodes_.size();
  KeyposeNode new_node(new_node_position, new_node_index, current_keypose_id_, false);
  new_node.SetCurrentKeyposePosition(current_keypose_position_);
  new_node.is_connected_ = true;
  nodes_.push_back(new_node);
  std::vector<int> neighbors;
  graph_.push_back(neighbors);
  std::vector<double> neighbor_dist;
  dist_.push_back(neighbor_dist);

  // 新增 节点
  //   添加 节点  给  Shared_Infor_ 添加  节点信息
  tare_planner::msg::SharedNode new_node_msg;
  new_node_msg.node_index = new_node_index;
  new_node_msg.node_position = new_node_position;
  new_node_msg.is_keypose = false;
  Shared_Infor_.new_node_set.push_back(new_node_msg);

  return new_node_index;
}

void KeyposeGraph::AddPath(const nav_msgs::msg::Path& path)
{
  if (path.poses.size() < 2)
  {
    return;
  }
  int prev_node_index = -1;
  for (int i = 0; i < path.poses.size(); i++)
  {
    int cur_node_index = AddNonKeyposeNode(path.poses[i].pose.position);
    if (i != 0)
    {
      // Add edge to previous node
      if (prev_node_index >= 0 && prev_node_index < nodes_.size())
      {
        // Check duplication
        if (!HasEdgeBetween(prev_node_index, cur_node_index))
        {
          geometry_msgs::msg::Point prev_node_position = nodes_[prev_node_index].position_;
          double dist_to_prev = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
              prev_node_position, path.poses[i].pose.position);
          graph_[prev_node_index].push_back(cur_node_index);
          graph_[cur_node_index].push_back(prev_node_index);

          dist_[prev_node_index].push_back(dist_to_prev);
          dist_[cur_node_index].push_back(dist_to_prev);

          // 添加边信息到共享数据
          tare_planner::msg::SharedEdge new_edge;
          if (prev_node_index > cur_node_index)
          {
            new_edge.node_index_min = cur_node_index;
            new_edge.node_index_max = prev_node_index;
          }
          else
          {
            new_edge.node_index_min = prev_node_index;
            new_edge.node_index_max = cur_node_index;
          }
          Shared_Infor_.add_edge_set.push_back(new_edge);
        }
      }
      else
      {
        // RCLCPP_ERROR_STREAM(rclcpp::get_logger("standalone_logger"), "KeyposeGraph::AddPath: prev_node_index "
        //                                                                  << prev_node_index << " out of bound [0, "
        //                                                                  << nodes_.size() - 1 << "]");
        return;
      }
    }
    prev_node_index = cur_node_index;
  }
  UpdateNodes();
}

void KeyposeGraph::GetMarker(visualization_msgs::msg::Marker& node_marker, visualization_msgs::msg::Marker& edge_marker)
{
  node_marker.points.clear();
  edge_marker.points.clear();

  for (const auto& node : nodes_)
  {
    node_marker.points.push_back(node.position_);
  }

  std::vector<std::pair<int, int>> added_edge;
  for (int i = 0; i < graph_.size(); i++)
  {
    int start_ind = i;
    for (int j = 0; j < graph_[i].size(); j++)
    {
      int end_ind = graph_[i][j];
      if (std::find(added_edge.begin(), added_edge.end(), std::make_pair(start_ind, end_ind)) == added_edge.end())
      {
        geometry_msgs::msg::Point start_node_position = nodes_[start_ind].position_;
        geometry_msgs::msg::Point end_node_position = nodes_[end_ind].position_;
        edge_marker.points.push_back(start_node_position);
        edge_marker.points.push_back(end_node_position);
        added_edge.emplace_back(start_ind, end_ind);
      }
    }
  }
}

int KeyposeGraph::GetConnectedNodeNum()
{
  int connected_node_num = 0;
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].is_connected_)
    {
      connected_node_num++;
    }
  }
  return connected_node_num;
}

void KeyposeGraph::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  cloud->clear();
  for (const auto& node : nodes_)
  {
    pcl::PointXYZI point;
    point.x = node.position_.x;
    point.y = node.position_.y;
    point.z = node.position_.z;
    if (node.is_connected_)
    {
      point.intensity = 10;
    }
    else
    {
      point.intensity = -1;
    }
    cloud->points.push_back(point);
  }
}

void KeyposeGraph::GetConnectedNodeIndices(int query_ind, std::vector<int>& connected_node_indices,
                                           std::vector<bool> constraints)
{
  if (nodes_.size() != constraints.size())
  {
    // RCLCPP_ERROR(rclcpp::get_logger("standalone_logger"),
    //              "KeyposeGraph::GetConnectedNodeIndices: constraints size not equal to node "
    //              "size");
    return;
  }
  if (query_ind < 0 || query_ind >= nodes_.size())
  {
    // RCLCPP_ERROR_STREAM(rclcpp::get_logger("standalone_logger"),
    //                     "KeyposeGraph::GetConnectedNodeIndices: query_ind: " << query_ind << " out of range: [0, "
    //                                                                          << nodes_.size() << "]");
    return;
  }
  connected_node_indices.clear();
  std::vector<bool> visited(nodes_.size(), false);
  std::stack<int> dfs_stack;
  dfs_stack.push(query_ind);
  while (!dfs_stack.empty())
  {
    int current_ind = dfs_stack.top();
    connected_node_indices.push_back(current_ind);
    dfs_stack.pop();
    if (!visited[current_ind])
    {
      visited[current_ind] = true;
    }
    for (int i = 0; i < graph_[current_ind].size(); i++)
    {
      int neighbor_ind = graph_[current_ind][i];
      if (!visited[neighbor_ind] && constraints[neighbor_ind])
      {
        dfs_stack.push(neighbor_ind);
      }
    }
  }
}

void KeyposeGraph::CheckLocalCollision(const geometry_msgs::msg::Point& robot_position,
                                       const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
{
  // Get local planning horizon xy size
  int in_local_planning_horizon_count = 0;
  int collision_node_count = 0;
  int collision_edge_count = 0;
  int in_viewpoint_range_count = 0;
  Eigen::Vector3d viewpoint_resolution = viewpoint_manager->GetResolution();
  double max_z_diff = std::max(viewpoint_resolution.x(), viewpoint_resolution.y()) * 2;
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].is_keypose_)
    {
      continue;
    }

    Eigen::Vector3d node_position =
        Eigen::Vector3d(nodes_[i].position_.x, nodes_[i].position_.y, nodes_[i].position_.z);
    int viewpoint_ind = viewpoint_manager->GetViewPointInd(node_position);
    bool node_in_collision = false;
    if (viewpoint_manager->InRange(viewpoint_ind) &&
        std::abs(viewpoint_manager->GetViewPointHeight(viewpoint_ind) - node_position.z()) < max_z_diff)
    {
      in_local_planning_horizon_count++;
      in_viewpoint_range_count++;
      if (viewpoint_manager->ViewPointInCollision(viewpoint_ind))
      {
        node_in_collision = true;
        collision_node_count++;
        // Delete all the associated edges
        for (int j = 0; j < graph_[i].size(); j++)
        {
          int neighbor_ind = graph_[i][j];
          for (int k = 0; k < graph_[neighbor_ind].size(); k++)
          {
            if (graph_[neighbor_ind][k] == i)
            {
              graph_[neighbor_ind].erase(graph_[neighbor_ind].begin() + k);
              dist_[neighbor_ind].erase(dist_[neighbor_ind].begin() + k);
              k--;

              // 记录要删除的边信息
              tare_planner::msg::SharedEdge new_edge;
              if (i > neighbor_ind)
              {
                new_edge.node_index_min = neighbor_ind;
                new_edge.node_index_max = i;
              }
              else
              {
                new_edge.node_index_min = i;
                new_edge.node_index_max = neighbor_ind;
              }
              Shared_Infor_.delete_edge_set.push_back(new_edge);

            }
          }
        }
        graph_[i].clear();
        dist_[i].clear();
      }
      else
      {
        Eigen::Vector3d viewpoint_resolution = viewpoint_manager->GetResolution();
        double collision_check_resolution = std::min(viewpoint_resolution.x(), viewpoint_resolution.y()) / 2;
        // Check edge collision
        for (int j = 0; j < graph_[i].size(); j++)
        {
          int neighbor_ind = graph_[i][j];
          Eigen::Vector3d start_position = node_position;
          Eigen::Vector3d end_position = Eigen::Vector3d(
              nodes_[neighbor_ind].position_.x, nodes_[neighbor_ind].position_.y, nodes_[neighbor_ind].position_.z);
          std::vector<Eigen::Vector3d> interp_points;
          misc_utils_ns::LinInterpPoints(start_position, end_position, collision_check_resolution, interp_points);
          for (const auto& collision_check_position : interp_points)
          {
            int viewpoint_ind = viewpoint_manager->GetViewPointInd(collision_check_position);
            if (viewpoint_manager->InRange(viewpoint_ind))
            {
              if (viewpoint_manager->ViewPointInCollision(viewpoint_ind))
              {
                geometry_msgs::msg::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);
                // Delete neighbors' edges
                for (int k = 0; k < graph_[neighbor_ind].size(); k++)
                {
                  if (graph_[neighbor_ind][k] == i)
                  {
                    collision_edge_count++;
                    graph_[neighbor_ind].erase(graph_[neighbor_ind].begin() + k);
                    dist_[neighbor_ind].erase(dist_[neighbor_ind].begin() + k);
                    k--;

                    // 记录要删除的边信息
                    tare_planner::msg::SharedEdge new_edge;
                    if (i > neighbor_ind)
                    {
                      new_edge.node_index_min = neighbor_ind;
                      new_edge.node_index_max = i;
                    }
                    else
                    {
                      new_edge.node_index_min = i;
                      new_edge.node_index_max = neighbor_ind;
                    }
                    Shared_Infor_.delete_edge_set.push_back(new_edge);
                  }
                }
                // Delete the node's edge
                graph_[i].erase(graph_[i].begin() + j);
                dist_[i].erase(dist_[i].begin() + j);
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

// 添加 使用 merger graph 的碰撞检测
void KeyposeGraph::CheckLocalCollisionMergerGraph(
    const geometry_msgs::msg::Point& robot_position,
    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
    std::vector<std::pair<int, int>>& delete_edge_set)
{
  // 获取本地规划范围xy大小
  int in_local_planning_horizon_count = 0;
  int collision_node_count = 0;
  int collision_edge_count = 0;
  int in_viewpoint_range_count = 0;
  Eigen::Vector3d viewpoint_resolution = viewpoint_manager->GetResolution();  // 视点分辨率
  double max_z_diff = std::max(viewpoint_resolution.x(), viewpoint_resolution.y()) * 2;
  
  for (size_t i = 0; i < nodes_.size(); i++)  // 遍历所有节点
  {
    if (nodes_[i].is_keypose_)
    {
      continue;  // 跳过关键位姿节点
    }
    
    // 处理非关键位姿节点
    Eigen::Vector3d node_position = Eigen::Vector3d(
        nodes_[i].position_.x,
        nodes_[i].position_.y,
        nodes_[i].position_.z
    );
    int viewpoint_ind = viewpoint_manager->GetViewPointInd(node_position);  // 获取节点在局部视点集中的编号
    bool node_in_collision = false;  // 初始化默认不在碰撞中
    
    if (viewpoint_manager->InRange(viewpoint_ind) &&
        std::abs(viewpoint_manager->GetViewPointHeight(viewpoint_ind) - node_position.z()) < max_z_diff)
    {
      // 节点在局部规划范围内且高度在阈值内
      in_local_planning_horizon_count++;
      in_viewpoint_range_count++;
      
      if (viewpoint_manager->ViewPointInCollision(viewpoint_ind))  // 节点处于碰撞中
      {
        node_in_collision = true;
        collision_node_count++;
        
        // 删除所有关联的边
        for (size_t j = 0; j < graph_[i].size(); j++)
        {
          int neighbor_ind = graph_[i][j];
          for (size_t k = 0; k < graph_[neighbor_ind].size(); k++)
          {
            if (graph_[neighbor_ind][k] == i)
            {
              graph_[neighbor_ind].erase(graph_[neighbor_ind].begin() + k);
              dist_[neighbor_ind].erase(dist_[neighbor_ind].begin() + k);
              k--;  // 调整索引
              
              // 记录要删除的边信息
              tare_planner::msg::SharedEdge new_edge;
              if (i > neighbor_ind)
              {
                new_edge.node_index_min = neighbor_ind;
                new_edge.node_index_max = i;
              }
              else
              {
                new_edge.node_index_min = i;
                new_edge.node_index_max = neighbor_ind;
              }
              Shared_Infor_.delete_edge_set.push_back(new_edge);

              // 添加到要删除的边集合
              delete_edge_set.emplace_back(i, neighbor_ind);
            }
          }
        }
        graph_[i].clear();
        dist_[i].clear();
      }
      else  // 节点不处于碰撞中，检查边碰撞
      {
        Eigen::Vector3d viewpoint_resolution = viewpoint_manager->GetResolution();
        double collision_check_resolution = std::min(viewpoint_resolution.x(), viewpoint_resolution.y()) / 2;

        // 检查边碰撞
        for (size_t j = 0; j < graph_[i].size(); j++)
        {
          int neighbor_ind = graph_[i][j];
          Eigen::Vector3d start_position = node_position;
          Eigen::Vector3d end_position = Eigen::Vector3d(
              nodes_[neighbor_ind].position_.x,
              nodes_[neighbor_ind].position_.y,
              nodes_[neighbor_ind].position_.z
          );

          std::vector<Eigen::Vector3d> interp_points;
          misc_utils_ns::LinInterpPoints(
              start_position,
              end_position,
              collision_check_resolution,
              interp_points
          );  // 生成插值点用于碰撞检测

          for (const auto& collision_check_position : interp_points)
          {
            int viewpoint_ind = viewpoint_manager->GetViewPointInd(collision_check_position);
            if (viewpoint_manager->InRange(viewpoint_ind))
            {
              if (viewpoint_manager->ViewPointInCollision(viewpoint_ind))  // 插值点处于碰撞中
              {
                geometry_msgs::msg::Point viewpoint_position =
                    viewpoint_manager->GetViewPointPosition(viewpoint_ind);

                // 删除邻居节点的边
                for (size_t k = 0; k < graph_[neighbor_ind].size(); k++)
                {
                  if (graph_[neighbor_ind][k] == i)
                  {
                    collision_edge_count++;
                    graph_[neighbor_ind].erase(graph_[neighbor_ind].begin() + k);
                    dist_[neighbor_ind].erase(dist_[neighbor_ind].begin() + k);
                    k--;  // 调整索引

                    // 记录要删除的边信息
                    tare_planner::msg::SharedEdge new_edge;
                    if (i > neighbor_ind)
                    {
                      new_edge.node_index_min = neighbor_ind;
                      new_edge.node_index_max = i;
                    }
                    else
                    {
                      new_edge.node_index_min = i;
                      new_edge.node_index_max = neighbor_ind;
                    }
                    Shared_Infor_.delete_edge_set.push_back(new_edge);
                    delete_edge_set.emplace_back(i, neighbor_ind);
                  }
                }
                
                // 删除当前节点的边
                graph_[i].erase(graph_[i].begin() + j);
                dist_[i].erase(dist_[i].begin() + j);
                j--;  // 调整索引
                break;
              }
            }
          }
        }
      }
    }
  }
}


void KeyposeGraph::UpdateNodes()
{
  nodes_cloud_->clear();
  for (int i = 0; i < nodes_.size(); i++)
  {
    pcl::PointXYZI point;
    point.x = nodes_[i].position_.x;
    point.y = nodes_[i].position_.y;
    point.z = nodes_[i].position_.z;
    point.intensity = i;
    nodes_cloud_->points.push_back(point);
  }
  if (!nodes_cloud_->points.empty())
  {
    kdtree_nodes_->setInputCloud(nodes_cloud_);
  }
}

void KeyposeGraph::CheckConnectivity(const geometry_msgs::msg::Point& robot_position)
{
  if (nodes_.empty())
  {
    return;
  }
  UpdateNodes();

  // The first keypose node is always connected, set all the others to be disconnected
  int first_keypose_node_ind = 0; // -1
  bool found_connected = false;

  // for (int i = 0; i < nodes_.size(); i++)
  // {
  //   if (nodes_[i].is_keypose_)
  //   {
  //     first_keypose_node_ind = i;
  //     break;
  //   }
  // }
  first_keypose_node_ind = new_keypose_ind_; // 新增

  // Check the connectivity starting from the robot
  for (int i = 0; i < nodes_.size(); i++)
  {
    nodes_[i].is_connected_ = false;
  }
  if (first_keypose_node_ind >= 0 && first_keypose_node_ind < nodes_.size())
  {
    nodes_[first_keypose_node_ind].is_connected_ = true;
    connected_node_indices_.clear();
    std::vector<bool> constraint(nodes_.size(), true);
    GetConnectedNodeIndices(first_keypose_node_ind, connected_node_indices_, constraint);
  }
  else
  {
    int robot_node_ind = -1;
    double robot_node_dist = DBL_MAX;
    GetClosestNodeIndAndDistance(robot_position, robot_node_ind, robot_node_dist);
    if (robot_node_ind >= 0 && robot_node_ind < nodes_.size())
    {
      nodes_[robot_node_ind].is_connected_ = true;
      connected_node_indices_.clear();
      std::vector<bool> constraint(nodes_.size(), true);
      GetConnectedNodeIndices(robot_node_ind, connected_node_indices_, constraint);
    }
    else
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("standalone_logger"),
                          "KeyposeGraph::CheckConnectivity: Cannot get closest robot node ind " << robot_node_ind);
    }
  }

  connected_nodes_cloud_->clear();
  for (int i = 0; i < connected_node_indices_.size(); i++)
  {
    int node_ind = connected_node_indices_[i];
    nodes_[node_ind].is_connected_ = true;
    pcl::PointXYZI point;
    point.x = nodes_[node_ind].position_.x;
    point.y = nodes_[node_ind].position_.y;
    point.z = nodes_[node_ind].position_.z;
    point.intensity = node_ind;
    connected_nodes_cloud_->points.push_back(point);
  }
  if (!connected_nodes_cloud_->points.empty())
  {
    kdtree_connected_nodes_->setInputCloud(connected_nodes_cloud_);
  }
}

int KeyposeGraph::AddKeyposeNode(const nav_msgs::msg::Odometry& keypose, const planning_env_ns::PlanningEnv& planning_env)
{
  current_keypose_position_ = keypose.pose.pose.position;
  current_keypose_id_ = static_cast<int>(keypose.pose.covariance[0]);
  int new_node_ind = nodes_.size();
  int keypose_node_count = 0;
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].is_keypose_)
    {
      keypose_node_count++;
    }
  }
  if (nodes_.empty() || keypose_node_count == 0)
  {
    AddNode(current_keypose_position_, new_node_ind, current_keypose_id_, true);
    new_keypose_ind_ = new_node_ind; // 新增
    return new_node_ind;
  }
  else
  {
    double min_dist = DBL_MAX;
    int min_dist_ind = -1;
    double last_keypose_dist = DBL_MAX;
    int last_keypose_ind = -1;
    int max_keypose_id = 0;
    std::vector<int> in_range_node_indices;
    std::vector<double> in_range_node_dist;
    for (int i = 0; i < nodes_.size(); i++)
    {
      if (!allow_vertical_edge_)
      {
        if (std::abs(nodes_[i].position_.z - current_keypose_position_.z) > kAddEdgeVerticalThreshold)
        {
          continue;
        }
      }
      double dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
          nodes_[i].position_, current_keypose_position_);
      if (dist < min_dist && nodes_[i].is_keypose_)
      {
        min_dist = dist;
        min_dist_ind = i;
      }
      int keypose_id = nodes_[i].keypose_id_;
      if (keypose_id > max_keypose_id && nodes_[i].is_keypose_)
      {
        last_keypose_dist = dist;
        last_keypose_ind = i;
        max_keypose_id = keypose_id;
      }
      if (dist < kAddEdgeConnectDistThr)
      {
        in_range_node_indices.push_back(i);
        in_range_node_dist.push_back(dist);
      }
    }
    // If the closest keypose node is some distance away
    if (min_dist_ind >= 0 && min_dist_ind < nodes_.size())
    {
      if (min_dist > kAddNodeMinDist)
      {
        // If the last keypose is within range
        if (last_keypose_dist < kAddEdgeToLastKeyposeDistThr && last_keypose_ind >= 0 &&
            last_keypose_ind < nodes_.size())
        {
          // Add edge to the last keypose node
          AddNodeAndEdge(current_keypose_position_, new_node_ind, current_keypose_id_, true, last_keypose_ind,
                         last_keypose_dist);
        }
        else
        {
          // Add edge to the nearest node
          AddNodeAndEdge(current_keypose_position_, new_node_ind, current_keypose_id_, true, min_dist_ind, min_dist);
        }
        // Check other nodes
        if (!in_range_node_indices.empty())
        {
          for (int idx = 0; idx < in_range_node_indices.size(); idx++)
          {
            int in_range_ind = in_range_node_indices[idx];
            if (in_range_ind >= 0 && in_range_ind < nodes_.size())
            {
              // Collision check
              KeyposeNode neighbor_node = nodes_[in_range_ind];
              if (std::find(graph_[new_node_ind].begin(), graph_[new_node_ind].end(), in_range_ind) !=
                  graph_[new_node_ind].end())
                continue;
              double neighbor_node_dist = in_range_node_dist[idx];
              double diff_x = neighbor_node.position_.x - current_keypose_position_.x;
              double diff_y = neighbor_node.position_.y - current_keypose_position_.y;
              double diff_z = neighbor_node.position_.z - current_keypose_position_.z;
              int check_point_num = static_cast<int>(neighbor_node_dist / kAddEdgeCollisionCheckResolution);
              bool in_collision = false;
              for (int i = 0; i < check_point_num; i++)
              {
                double check_point_x =
                    current_keypose_position_.x + kAddEdgeCollisionCheckResolution * i * diff_x / neighbor_node_dist;
                double check_point_y =
                    current_keypose_position_.y + kAddEdgeCollisionCheckResolution * i * diff_y / neighbor_node_dist;
                double check_point_z =
                    current_keypose_position_.z + kAddEdgeCollisionCheckResolution * i * diff_z / neighbor_node_dist;
                if (planning_env.InCollision(check_point_x, check_point_y, check_point_z))
                {
                  in_collision = true;
                  break;
                }
              }
              if (!in_collision)
              {
                AddEdge(new_node_ind, in_range_ind, neighbor_node_dist);
              }
            }
          }
        }
        new_keypose_ind_ = new_node_ind;
        return new_node_ind;
      }
      else
      {
        new_keypose_ind_ = min_dist_ind;
        return min_dist_ind;
      }
    }
    else
    {
      // RCLCPP_ERROR_STREAM(rclcpp::get_logger("standalone_logger"),
      //                     "KeyposeGraph::AddKeyposeNode: Nearest keypose ind out of range: " << min_dist_ind);
      new_keypose_ind_ = new_node_ind; // 新增
      return new_node_ind;
    }
  }
}

bool KeyposeGraph::IsPositionReachable(const geometry_msgs::msg::Point& point, double dist_threshold)
{
  int closest_node_ind = 0;
  double closest_node_dist = DBL_MAX;
  GetClosestConnectedNodeIndAndDistance(point, closest_node_ind, closest_node_dist);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size() && closest_node_dist < dist_threshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool KeyposeGraph::IsPositionReachable(const geometry_msgs::msg::Point& point)
{
  int closest_node_ind = 0;
  double closest_node_dist = DBL_MAX;
  GetClosestConnectedNodeIndAndDistance(point, closest_node_ind, closest_node_dist);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size() && closest_node_dist < kAddNonKeyposeNodeMinDist)
  {
    return true;
  }
  else
  {
    return false;
  }
}

int KeyposeGraph::GetClosestNodeInd(const geometry_msgs::msg::Point& point)
{
  int node_ind = 0;
  double min_dist = DBL_MAX;
  GetClosestNodeIndAndDistance(point, node_ind, min_dist);
  return node_ind;
}

void KeyposeGraph::GetClosestNodeIndAndDistance(const geometry_msgs::msg::Point& point, int& node_ind, double& dist)
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
    // RCLCPP_WARN_STREAM(rclcpp::get_logger("standalone_logger"),
    //                    "KeyposeGraph::GetClosestNodeIndAndDistance: search for nearest neighbor failed with "
    //                        << nodes_cloud_->points.size() << " nodes.");
    if (!nearest_neighbor_node_indices.empty())
    {
      // RCLCPP_WARN_STREAM(rclcpp::get_logger("standalone_logger"),
      //                    "Nearest neighbor node Ind: " << nearest_neighbor_node_indices.front());
    }
    for (int i = 0; i < nodes_.size(); i++)
    {
      geometry_msgs::msg::Point node_position = nodes_[i].position_;
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

void KeyposeGraph::GetClosestConnectedNodeIndAndDistance(const geometry_msgs::msg::Point& point, int& node_ind, double& dist)
{
  if (connected_nodes_cloud_->points.empty())
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
    // RCLCPP_WARN_STREAM(rclcpp::get_logger("standalone_logger"),
    //                    "KeyposeGraph::GetClosestNodeInd: search for nearest neighbor failed with "
    //                        << connected_nodes_cloud_->points.size() << " connected nodes.");
    node_ind = -1;
    dist = 0;
  }
}

int KeyposeGraph::GetClosestKeyposeID(const geometry_msgs::msg::Point& point)
{
  int closest_node_ind = GetClosestNodeInd(point);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
  {
    return nodes_[closest_node_ind].keypose_id_;
  }
  else
  {
    return -1;
  }
}

geometry_msgs::msg::Point KeyposeGraph::GetClosestNodePosition(const geometry_msgs::msg::Point& point)
{
  int closest_node_ind = GetClosestNodeInd(point);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
  {
    return nodes_[closest_node_ind].position_;
  }
  else
  {
    geometry_msgs::msg::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    return point;
  }
}

bool KeyposeGraph::GetShortestPathWithMaxLength(const geometry_msgs::msg::Point& start_point,
                                                const geometry_msgs::msg::Point& target_point, double max_path_length,
                                                bool get_path, nav_msgs::msg::Path& path)
{
  if (nodes_.size() < 2)
  {
    if (get_path)
    {
      geometry_msgs::msg::PoseStamped start_pose;
      start_pose.pose.position = start_point;
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.pose.position = target_point;
      path.poses.push_back(start_pose);
      path.poses.push_back(target_pose);
    }
    return misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(start_point, target_point);
  }
  int from_idx = 0;
  int to_idx = 0;
  double min_dist_to_start = DBL_MAX;
  double min_dist_to_target = DBL_MAX;
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (allow_vertical_edge_)
    {
      double dist_to_start = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_, start_point);
      double dist_to_target = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_, target_point);
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        from_idx = i;
      }
      if (dist_to_target < min_dist_to_target)
      {
        min_dist_to_target = dist_to_target;
        to_idx = i;
      }
    }
    else
    {
      double z_diff_to_start = std::abs(nodes_[i].position_.z - start_point.z);
      double z_diff_to_target = std::abs(nodes_[i].position_.z - target_point.z);
      // TODO: parameterize this
      if (z_diff_to_start < 1.5)
      {
        double xy_dist_to_start = misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_, start_point);
        if (xy_dist_to_start < min_dist_to_start)
        {
          min_dist_to_start = xy_dist_to_start;
          from_idx = i;
        }
      }
      if (z_diff_to_target < 1.5)
      {
        double xy_dist_to_target = misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_, target_point);
        if (xy_dist_to_target < min_dist_to_target)
        {
          min_dist_to_target = xy_dist_to_target;
          to_idx = i;
        }
      }
    }
  }

  std::vector<geometry_msgs::msg::Point> node_positions;
  for (int i = 0; i < nodes_.size(); i++)
  {
    node_positions.push_back(nodes_[i].position_);
  }
  std::vector<int> path_indices;
  double shortest_dist = DBL_MAX;
  bool found_path = misc_utils_ns::AStarSearchWithMaxPathLength(graph_, dist_, node_positions, from_idx, to_idx,
                                                                get_path, path_indices, shortest_dist, max_path_length);
  if (found_path && get_path)
  {
    path.poses.clear();
    for (const auto& ind : path_indices)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position = nodes_[ind].position_;
      pose.pose.orientation.w = nodes_[ind].keypose_id_;
      pose.pose.orientation.x = ind;
      path.poses.push_back(pose);
    }
  }

  return found_path;
}

double KeyposeGraph::GetShortestPath(const geometry_msgs::msg::Point& start_point,
                                     const geometry_msgs::msg::Point& target_point, bool get_path,
                                     nav_msgs::msg::Path& path, bool use_connected_nodes)
{
  if (nodes_.size() < 2)
  {
    if (get_path)
    {
      geometry_msgs::msg::PoseStamped start_pose;
      start_pose.pose.position = start_point;
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.pose.position = target_point;
      path.poses.push_back(start_pose);
      path.poses.push_back(target_pose);
    }
    return misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(start_point, target_point);
  }
  int from_idx = 0;
  int to_idx = 0;
  double min_dist_to_start = DBL_MAX;
  double min_dist_to_target = DBL_MAX;
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (use_connected_nodes && !nodes_[i].is_connected_)
    {
      continue;
    }
    if (allow_vertical_edge_)
    {
      double dist_to_start = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_, start_point);
      double dist_to_target = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_, target_point);
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        from_idx = i;
      }
      if (dist_to_target < min_dist_to_target)
      {
        min_dist_to_target = dist_to_target;
        to_idx = i;
      }
    }
    else
    {
      double z_diff_to_start = std::abs(nodes_[i].position_.z - start_point.z);
      double z_diff_to_target = std::abs(nodes_[i].position_.z - target_point.z);
      // TODO: parameterize this
      if (z_diff_to_start < 1.5)
      {
        double xy_dist_to_start = misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_, start_point);
        if (xy_dist_to_start < min_dist_to_start)
        {
          min_dist_to_start = xy_dist_to_start;
          from_idx = i;
        }
      }
      if (z_diff_to_target < 1.5)
      {
        double xy_dist_to_target = misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_, target_point);
        if (xy_dist_to_target < min_dist_to_target)
        {
          min_dist_to_target = xy_dist_to_target;
          to_idx = i;
        }
      }
    }
  }
  //如果目标点  和关键位姿图上的点     太远，则不返回
  if (min_dist_to_target > kLocal_planning_radius)  //目标距离最近的关键位姿图上的结点      也大于局部规划的一半
                                                    //则，这个目标点是外部点未探索点
  {
    return -1;
  }

  std::vector<geometry_msgs::msg::Point> node_positions;
  // A*
  for (int i = 0; i < nodes_.size(); i++)
  {
    node_positions.push_back(nodes_[i].position_);
  }
  std::vector<int> path_indices;
  double shortest_dist =
      misc_utils_ns::AStarSearch(graph_, dist_, node_positions, from_idx, to_idx, get_path, path_indices);
  if (get_path)
  {
    path.poses.clear();
    for (const auto& ind : path_indices)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position = nodes_[ind].position_;
      pose.pose.orientation.w = nodes_[ind].keypose_id_;
      pose.pose.orientation.x = ind;
      path.poses.push_back(pose);
    }
  }

  return shortest_dist;
}

bool KeyposeGraph::GetShortestPathTwoPoint(const geometry_msgs::msg::Point& start_point,
                                           const geometry_msgs::msg::Point& target_point, bool get_path,
                                           nav_msgs::msg::Path& path, bool use_connected_nodes, double& kcellsize)
{
  if (nodes_.size() < 2)
  {
    if (get_path)
    {
      geometry_msgs::msg::PoseStamped start_pose;
      start_pose.pose.position = start_point;
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.pose.position = target_point;
      path.poses.push_back(start_pose);
      path.poses.push_back(target_pose);
    }
    return false;
  }
  
  int start_closest_node_ind = 0;
  double start_closest_node_dist = DBL_MAX;
  int goal_closest_node_ind = 0;
  double goal_closest_node_dist = DBL_MAX;
  geometry_msgs::msg::Point start_point_;
  geometry_msgs::msg::Point goal_point_;
  bool flag_start = false;
  bool flag_goal = false;
  
  // 使用kd树查找最近点
  GetClosestConnectedNodeIndAndDistance(start_point, start_closest_node_ind,
                                        start_closest_node_dist);  // 在关键位姿图上找到起点最近的节点和距离
  
  if (start_closest_node_dist < kcellsize && start_closest_node_ind >= 0 &&
      start_closest_node_ind < GetNodeNum())  // 检查节点有效性
  {
    start_point_ = GetNodePosition(start_closest_node_ind);  // 获取起点在图中的位置
    flag_start = true;
    // RCLCPP_INFO(nh->get_logger(), "存在 start_point_");
  }
  
  GetClosestConnectedNodeIndAndDistance(target_point, goal_closest_node_ind,
                                        goal_closest_node_dist);  // 在关键位姿图上找到目标点最近的节点和距离
  
  if (goal_closest_node_dist < kcellsize && goal_closest_node_ind >= 0 &&
      goal_closest_node_ind < GetNodeNum())  // 检查节点有效性
  {
    goal_point_ = GetNodePosition(goal_closest_node_ind);  // 获取目标点在图中的位置
    flag_goal = true;
    // RCLCPP_INFO(nh->get_logger(), "存在 goal_point_");
  }
  
  // 若两点中有一点不在图中，则返回false
  if (!flag_goal || !flag_start)
  {
    return false;
  }
  
  // 计算路径
  std::vector<geometry_msgs::msg::Point> node_positions;
  for (size_t i = 0; i < nodes_.size(); i++)  // 使用size_t更安全
  {
    node_positions.push_back(nodes_[i].position_);
  }
  
  std::vector<int> path_indices;
  double shortest_dist = misc_utils_ns::AStarSearch(
      graph_, dist_, node_positions, 
      start_closest_node_ind, goal_closest_node_ind, 
      get_path, path_indices
  );
  
  if (path_indices.size() < 2)
  {
    // RCLCPP_WARN(node_->get_logger(), "路径数小于2  出错");
    return false;
  }
  
  if (get_path)  // 如果需要路径，则填充路径消息
  {
    path.poses.clear();
    for (const auto& ind : path_indices)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position = nodes_[ind].position_;
      pose.pose.orientation.w = nodes_[ind].keypose_id_;
      pose.pose.orientation.x = ind;
      path.poses.push_back(pose);
    }
  }
  
  return true;  // 查找成功
}

geometry_msgs::msg::Point KeyposeGraph::GetFirstKeyposePosition()
{
  geometry_msgs::msg::Point point;
  point.x = 0;
  point.y = 0;
  point.z = 0;
  for (const auto& node : nodes_)
  {
    if (node.IsKeypose())
    {
      point = node.position_;
      break;
    }
  }
  return point;
}

geometry_msgs::msg::Point KeyposeGraph::GetKeyposePosition(int keypose_id)
{
  geometry_msgs::msg::Point point;
  point.x = 0;
  point.y = 0;
  point.z = 0;
  for (const auto& node : nodes_)
  {
    if (node.keypose_id_ == keypose_id)
    {
      point = node.position_;
      break;
    }
  }
  return point;
}

void KeyposeGraph::GetKeyposePositions(std::vector<Eigen::Vector3d>& positions)
{
  positions.clear();
  for (const auto& node : nodes_)
  {
    if (node.IsKeypose())
    {
      Eigen::Vector3d position(node.position_.x, node.position_.y, node.position_.z);
      positions.push_back(position);
    }
  }
}

geometry_msgs::msg::Point KeyposeGraph::GetNodePosition(int node_ind)
{
  geometry_msgs::msg::Point node_position;
  node_position.x = 0;
  node_position.y = 0;
  node_position.z = 0;
  if (node_ind >= 0 && node_ind < nodes_.size())
  {
    node_position = nodes_[node_ind].position_;
  }
  else
  {
    // RCLCPP_WARN_STREAM(rclcpp::get_logger("standalone_logger"), "KeyposeGraph::GetNodePosition: node_ind "
    //                                                                 << node_ind << " out of bound [0, "
    //                                                                 << nodes_.size() - 1 << "]");
  }
  return node_position;
}

}  // namespace keypose_graph_ns
