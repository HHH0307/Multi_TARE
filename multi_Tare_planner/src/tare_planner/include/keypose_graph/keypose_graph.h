//
// Created by caochao on 12/31/19.
//

#ifndef SENSOR_COVERAGE_PLANNER_KEYPOSE_GRAPH_H
#define SENSOR_COVERAGE_PLANNER_KEYPOSE_GRAPH_H

#include <functional>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <planning_env/planning_env.h>
#include <utils/misc_utils.h>

// 新增  共享类型
#include "tare_planner/msg/shared_merger_infor.hpp"

namespace viewpoint_manager_ns
{
class ViewPointManager;
}

namespace keypose_graph_ns
{
struct KeyposeNode; // 结点数据结构体
class KeyposeGraph; // 图数据类对象
}  // namespace keypose_graph_ns

struct keypose_graph_ns::KeyposeNode
{
  geometry_msgs::msg::Point position_;
  geometry_msgs::msg::Point offset_to_keypose_;
  int keypose_id_;
  int node_ind_;
  int cell_ind_;
  bool is_keypose_;
  bool is_connected_;
  int robot_id_;                                 // 所属机器人ID（拼接图用）

public:
  explicit KeyposeNode(double x = 0, double y = 0, double z = 0, int node_ind = 0, int keypose_id = 0, bool is_keypose = true);
  explicit KeyposeNode(const geometry_msgs::msg::Point& point, int node_ind = 0, int keypose_id = 0, bool is_keypose = true);
  ~KeyposeNode() = default;
  bool IsKeypose() const
  {
    return is_keypose_;
  }
  bool IsConnected() const
  {
    return is_connected_;
  }
  void SetOffsetToKeypose(const geometry_msgs::msg::Point& offset_to_keypose)
  {
    offset_to_keypose_ = offset_to_keypose;
  }
  void SetCurrentKeyposePosition(const geometry_msgs::msg::Point& current_keypose_position)
  {
    offset_to_keypose_.x = position_.x - current_keypose_position.x;
    offset_to_keypose_.y = position_.y - current_keypose_position.y;
    offset_to_keypose_.z = position_.z - current_keypose_position.z;
  }
};

class keypose_graph_ns::KeyposeGraph
{
private:
  bool allow_vertical_edge_;
  int current_keypose_id_;
  int new_keypose_ind_;  //  最新的  当前  点  索引
  geometry_msgs::msg::Point current_keypose_position_;
  std::vector<std::vector<int>> graph_;
  std::vector<std::vector<double>> dist_;
  std::vector<KeyposeNode> nodes_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_connected_nodes_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr connected_nodes_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_nodes_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr nodes_cloud_;

  // 共享节点信息
  tare_planner::msg::SharedMergerInfor Shared_Infor_;  // 共享信息
  std::vector<int> connected_node_indices_;

  double kAddNodeMinDist;
  double kAddNonKeyposeNodeMinDist;
  double kAddEdgeConnectDistThr;
  double kAddEdgeToLastKeyposeDistThr;
  double kAddEdgeVerticalThreshold;
  double kAddEdgeCollisionCheckResolution;
  double kAddEdgeCollisionCheckRadius;
  int kAddEdgeCollisionCheckPointNumThr;
  double kLocal_planning_radius;             // 局部规划半径

public:
  // 默认构造函数
  KeyposeGraph();
  KeyposeGraph(rclcpp::Node::SharedPtr nh);
  ~KeyposeGraph() = default;
  void ReadParameters(rclcpp::Node::SharedPtr nh);
  void AddNode(const geometry_msgs::msg::Point& position, int node_ind, int keypose_id, bool is_keypose);
  void AddNode(const geometry_msgs::msg::Point& position, int node_ind, int keypose_id, bool is_keypose, int robot_id);  // 重载
  void AddNodeAndEdge(const geometry_msgs::msg::Point& position, int node_ind, int keypose_id, bool is_keypose, int connected_node_ind, double connected_node_dist);
  void AddEdge(int from_node_ind, int to_node_ind, double dist);
  bool HasNode(const Eigen::Vector3d& position);
  bool InBound(int index)
  {
    return index >= 0 && index < graph_.size();
  }
  int GetNodeNum()
  {
    return nodes_.size();
  }
  int GetConnectedNodeNum();
  void GetMarker(visualization_msgs::msg::Marker& node_marker, visualization_msgs::msg::Marker& edge_marker);
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
  std::vector<int> GetConnectedGraphNodeIndices()
  {
    return connected_node_indices_;
  }
  void GetConnectedNodeIndices(int query_ind, std::vector<int>& connected_node_indices, std::vector<bool> constraints);
  void CheckLocalCollision(const geometry_msgs::msg::Point& robot_position,
                           const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager);
  // 使用拼接图的碰撞检测，获取删除边信息
  void CheckLocalCollisionMergerGraph(const geometry_msgs::msg::Point& robot_position,
                                      const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                      std::vector<std::pair<int, int>>& delete_edge_set);
  void UpdateNodes();
  void CheckConnectivity(const geometry_msgs::msg::Point& robot_position);
  int AddKeyposeNode(const nav_msgs::msg::Odometry& keypose, const planning_env_ns::PlanningEnv& planning_env);
  bool HasEdgeBetween(int node_ind1, int node_ind2);
  bool IsConnected(const Eigen::Vector3d& from_position, const Eigen::Vector3d& to_position);
  int AddNonKeyposeNode(const geometry_msgs::msg::Point& new_node_position);
  void AddPath(const nav_msgs::msg::Path& path);
  void SetAllowVerticalEdge(bool allow_vertical_edge)
  {
    allow_vertical_edge_ = allow_vertical_edge;
  }
  bool IsPositionReachable(const geometry_msgs::msg::Point& point, double dist_threshold);
  bool IsPositionReachable(const geometry_msgs::msg::Point& point);
  int GetClosestNodeInd(const geometry_msgs::msg::Point& point);
  void GetClosestNodeIndAndDistance(const geometry_msgs::msg::Point& point, int& node_ind, double& dist);
  void GetClosestConnectedNodeIndAndDistance(const geometry_msgs::msg::Point& point, int& node_ind, double& dist);
  int GetClosestKeyposeID(const geometry_msgs::msg::Point& point);
  geometry_msgs::msg::Point GetClosestNodePosition(const geometry_msgs::msg::Point& point);
  bool GetShortestPathWithMaxLength(const geometry_msgs::msg::Point& start_point, const geometry_msgs::msg::Point& target_point,
                                    double max_path_length, bool get_path, nav_msgs::msg::Path& path);
  double GetShortestPath(const geometry_msgs::msg::Point& start_point, const geometry_msgs::msg::Point& target_point,
                         bool get_path, nav_msgs::msg::Path& path, bool use_connected_nodes = false);
  // 两点间最短路径（新增函数）
  bool GetShortestPathTwoPoint(const geometry_msgs::msg::Point& start_point, const geometry_msgs::msg::Point& target_point,
                               bool get_path, nav_msgs::msg::Path& path, bool use_connected_nodes, double& kcellsize);

  double& SetAddNodeMinDist()
  {
    return kAddNodeMinDist;
  }
  double& SetAddNonKeyposeNodeMinDist()
  {
    return kAddNonKeyposeNodeMinDist;
  }
  double& SetAddEdgeCollisionCheckResolution()
  {
    return kAddEdgeCollisionCheckResolution;
  }
  double& SetAddEdgeCollisionCheckRadius()
  {
    return kAddEdgeCollisionCheckRadius;
  }
  int& SetAddEdgeCollisionCheckPointNumThr()
  {
    return kAddEdgeCollisionCheckPointNumThr;
  }
  double& SetAddEdgeConnectDistThr()
  {
    return kAddEdgeConnectDistThr;
  }
  double& SetAddEdgeToLastKeyposeDistThr()
  {
    return kAddEdgeToLastKeyposeDistThr;
  }
  double& SetAddEdgeVerticalThreshold()
  {
    return kAddEdgeVerticalThreshold;
  }
  geometry_msgs::msg::Point GetFirstKeyposePosition();
  geometry_msgs::msg::Point GetKeyposePosition(int keypose_id);
  void GetKeyposePositions(std::vector<Eigen::Vector3d>& positions);
  geometry_msgs::msg::Point GetNodePosition(int node_ind);

  // 清空共享信息
  void ClearSharedInfor() {
    Shared_Infor_.new_node_set.clear();
    Shared_Infor_.add_edge_set.clear();
    Shared_Infor_.delete_edge_set.clear();
  }

  // 获取共享信息
  tare_planner::msg::SharedMergerInfor GetSharedInfor() {
    return Shared_Infor_;
  }
  
  // 获取新节点索引
  int GetNewNodeIndex() 
  { 
    return nodes_.size(); 
  }

  void DeleteEdge(int node_1, int node_2)
  {
    for (size_t j = 0; j < graph_[node_1].size(); j++)
    {
      if (graph_[node_1][j] == node_2)
      {
        graph_[node_1].erase(graph_[node_1].begin() + j);
        dist_[node_1].erase(dist_[node_1].begin() + j);
        j--;
      }
    }
    for (size_t k = 0; k < graph_[node_2].size(); k++)
    {
      if (graph_[node_2][k] == node_1)
      {
        graph_[node_2].erase(graph_[node_2].begin() + k);
        dist_[node_2].erase(dist_[node_2].begin() + k);
        k--;
      }
    }
  }
  

  // 获取不连通点云
  pcl::PointCloud<pcl::PointXYZI> GetDisConnectNodes() {
    pcl::PointCloud<pcl::PointXYZI> disconnect_nodes;
    for (size_t i = 0; i < nodes_.size(); i++) {
      if (!nodes_[i].is_connected_) {
        pcl::PointXYZI temp_point;
        temp_point.x = nodes_[i].position_.x;
        temp_point.y = nodes_[i].position_.y;
        temp_point.z = nodes_[i].position_.z;
        temp_point.intensity = nodes_[i].node_ind_;
        disconnect_nodes.push_back(temp_point);
      }
    }
    return disconnect_nodes;
  }

  // 获取不连通点云的索引
  std::vector<int> GetDisConnectedNodeIndes() {
    std::vector<int> disconnected_nodes_index;
    for (size_t i = 0; i < nodes_.size(); i++) {
      if (!nodes_[i].is_connected_) {
        disconnected_nodes_index.push_back(nodes_[i].node_ind_);
      }
    }
    return disconnected_nodes_index;
  }

  // 节点操作函数
  bool GetNodeIsKeypose(int index) {
    return nodes_[index].is_keypose_;
  }

  void SetNodeIsConnected(int index, bool is_connected = true) {
    nodes_[index].is_connected_ = is_connected;
  }

  void ClearConnectedNodeIndices() {
    connected_node_indices_.clear();
  }

  void SetConnectedNodeIndices(std::vector<int>& connected_node_indices) {
    connected_node_indices_ = connected_node_indices;
  }
  
  int GetConnectedNodeIndicesNum() {
    return connected_node_indices_.size();
  }

  // 获取图和距离矩阵
  std::vector<std::vector<int>>& GetGraph() 
  { 
    return graph_; 
  }
  std::vector<std::vector<double>>& GetDist() 
  { 
    return dist_; 
  }

  // 获取参数值
  double GetkAddNonKeyposeNodeMinDist() 
  { 
    return kAddNonKeyposeNodeMinDist; 
  }
  
  // 获取节点所属机器人ID
  int GetNodeRobotId(int index) 
  { 
    return nodes_[index].robot_id_; 
  }
  
  // 获取节点是否连通
  bool GetNodeIsConnected(int index) 
  { 
    return nodes_[index].IsConnected(); 
  }
  
  // 设置连接节点索引
  void SetConnectedNodeIndices() {
    for (int node_ind : connected_node_indices_) {
      nodes_[node_ind].is_connected_ = true;
    }
  }
};

#endif  // SENSOR_COVERAGE_PLANNER_KEYPOSE_GRAPH_H
