/**
 * @file merger_graph.h
 * @author ljj
 * @brief  仿照  keypose_graph   做    一个  拼接  图   使用 ikdtree    做搜索
 * @version 0.1
 * @date 2022-11-22
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef MERGER_GRAPH_H_
#define MERGER_GRAPH_H_
#include "keypose_graph/keypose_graph.h"
#include "rclcpp/rclcpp.hpp"
// #include "ikd_Tree.h"
#include "ikd-Tree/ikd_Tree_impl.h"
// #include "viewpoint/viewpoint.h"
#include <unordered_map>
// 共享信息
// #include "geometry_msgs/msg/point.hpp"
// #include "nav_msgs/msg/path.hpp"
#include "tare_planner/msg/merger_graph_edge.hpp"

namespace viewpoint_manager_ns
{
class ViewPointManager;
}

namespace merger_graph_ns
{
//  继承 keypose graph   修改  kd树搜索  和 增加 索引替换
class MergerGraph : public keypose_graph_ns::KeyposeGraph
{
public:
  //构造函数
  MergerGraph(rclcpp::Node::SharedPtr nh);
  ~MergerGraph() = default;

  // 处理  一次  接受数据 函数
  void ProcessOneceData(std::unordered_map<int, tare_planner::msg::SharedMergerInfor>& Shared_Infor_map);

  void ProcessOneceDataAddEdge(std::unordered_map<int, tare_planner::msg::SharedMergerInfor>& Shared_Infor_map);

  // ikdtree   connected_nodes  维护
  void MaintainConnectedNodesIkdtree();

  // 重写kd树相关的 函数   使用  ikd  替换

  // 得到 最近节点  和 距离  在所有节点上搜索
  void GetClosestNodeIndAndDistanceIkdtree(const geometry_msgs::msg::Point& point, int& node_ind, double& dist);
  // 得到  最近节点  和 距离 在  连接节点  上搜索
  void GetClosestConnectedNodeIndAndDistanceIkdtree(const geometry_msgs::msg::Point& point, int& node_ind, double& dist);
  // 传入 机器人位置  范围  获得距离
  void GetRangeClosestConnectedNodeIndIkdtree(const geometry_msgs::msg::Point& point, float range,
                                              std::vector<int>& node_indices);

  //重写 HasNode  判定
  bool HasNodeIkdtree(const Eigen::Vector3d& position);
  //重写 两个位置 是否连接
  bool IsConnctedIkdtree(const Eigen::Vector3d& from_position, const Eigen::Vector3d& to_position);
  // 重写  检查 连通性
  void CheckConnectivityIkdtree(const geometry_msgs::msg::Point& robot_position);
  // 重写 最近节点  index
  int GetClosestNodeIndIkdtree(const geometry_msgs::msg::Point& point);

  // 重写  位置是否可以到达  两个参数
  bool IsPositionReachableIkdtree(const geometry_msgs::msg::Point& point, double dist_threshold);
  // 重写  位置是否可以到达  单参数
  bool IsPositionReachableIkdtree(const geometry_msgs::msg::Point& point);
  // 重写  求最短路径
  bool GetShortestPathTwoPointIkdtree(const geometry_msgs::msg::Point& start_point, const geometry_msgs::msg::Point& target_point,
                                      bool get_path, nav_msgs::msg::Path& path, bool use_connected_nodes, double& kcellsize);

  //操作     新增   索引的  映射   到       robot_keyposeind_mergerind_
  void AddMap2robtkeyposeindmergerind(int robot_id, int keypose_ind, int merger_graph_ind)
  {
    // 添加点   多机器人到   拼接图的索引
    robot_keyposeind_mergerind_[robot_id][keypose_ind] = merger_graph_ind;
    // 配套操作    添加反向 索引
    if (mergerind_robot_keyposeind_.size() == merger_graph_ind)
    {
      std::map<int, int> temp;
      temp[robot_id] = keypose_ind;
      mergerind_robot_keyposeind_.push_back(temp);
    }
    else if (merger_graph_ind >= 0 && merger_graph_ind < mergerind_robot_keyposeind_.size())
    {
      // 说明是  融和节点  共享 同一个   索引  merger    index
      mergerind_robot_keyposeind_[merger_graph_ind].insert(std::pair<int, int>(robot_id, keypose_ind));
    }
  }

  // 添加 函数

  void AddEdgeLocalRobotWithOtherRobotNodesInRange(
      const geometry_msgs::msg::Point& start_point, float range, int robot_id, int cur_keypose_ind,
      const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
      std::vector<std::pair<int, int>>& add_edge);

  void Addfusion_node_num()
  {
    fusion_node_num++;
  }
  void Addsame_subgrid_edge_num()
  {
    same_subgrid_edge_num++;
  }
  void Addtrajectory_edd_num()
  {
    trajectory_edd_num++;
  }

  void UpdataOnceMergerGraphEdge(tare_planner::msg::MergerGraphEdge& shared_merger_graph_edge ,
                                                                            const std::vector<std::pair<int,int>>& add_edge,
                                                                            const std::vector<std::pair<int,int>>& delete_edge);
  void UpdataOnceMergerGraphEdgeFromOthers(tare_planner::msg::MergerGraphEdge& MergerGraphEdge_Once_sub);

private:
  rclcpp::Node::SharedPtr node_;  // 添加node_成员声明
  // 添加  ikdtree  将  所有节点都插入进来 ikdtree  只维护能够通行的节点  还是所有节点
  bool ikd_init_;                           // ikd 树初始化操作
  KD_TREE<pcl::PointXYZI>::Ptr ikd_nodes_;  // 点使用  XYZI  其中  I  是当前图的 索引
  KD_TREE<pcl::PointXYZI>::Ptr ikd_connected_nodes_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ikd_add_cloud_;  // 递增  增加节点
  // pcl::PointCloud<pcl::PointXYZI>::Ptr ikd_delete_cloud_;  // 不连接的 删除节点    没用上  直接在局部 使用点处理了

  //   增量式  删除  保留    当前  删除的  编号
  std::vector<int> disconnected_nodes_ind_;

  // 用于查找边关系  索引  机器人  id  keypose_graph  索引   与  当前  拼接图的  索引
  // 使用  vector  应该也可以  毕竟 keypose_graph  编号  按照 顺序增加的。
  // 网上说  100 以内 map查找效率高于unoredered_map  机器人个数小于100    节点个数  1000～10000 的数量级
  std::map<int, std::unordered_map<int, int>> robot_keyposeind_mergerind_;
  // 反向 索引     merger_node  _  robot _ node
  //  使用  vector        地址      就是      索引号
  //   在添加  节点  的位置  添加 反向  索引   用于   边更新 的  共享
  std::vector<std::map<int, int>> mergerind_robot_keyposeind_;


  int current_mergerpose_id_;  //当前的  机器人位姿     的索引

  // 融合点个数
  int fusion_node_num;
  // 添加同网格内不同机器人的边个数
  int same_subgrid_edge_num;
  // 添加机器人轨迹附近不同机器人的边的个数
  int trajectory_edd_num;
};

};  // namespace merger_graph_ns

#endif
