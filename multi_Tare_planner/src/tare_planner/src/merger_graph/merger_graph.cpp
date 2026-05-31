#include <merger_graph/merger_graph.h>
#include <viewpoint_manager/viewpoint_manager.h>

namespace merger_graph_ns
{
// 构造函数
MergerGraph::MergerGraph(rclcpp::Node::SharedPtr nh) : KeyposeGraph(nh), node_(nh) // 调用父类构造函数
{
  ReadParameters(nh);
  // 调用函数的方式使用  keypose_graph  的变量
  ikd_nodes_ = KD_TREE<pcl::PointXYZI>::Ptr(new KD_TREE<pcl::PointXYZI>(
      0.5, 0.6, 0.1));  // 使用  参数  0.5  的删除  0.6 的平衡参数 0.1 的下采样  但整体不做下采样
  ikd_connected_nodes_ = KD_TREE<pcl::PointXYZI>::Ptr(new KD_TREE<pcl::PointXYZI>(
      0.5, 0.6, 0.1));  // 使用  参数  0.5  的删除  0.6 的平衡参数 0.1 的下采样  但整体不做下采样
  //  ikd_add_cloud_ 是两棵树都添加   ikd_delete_cloud_ 是在连通树上做删除
  ikd_add_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  // ikd_delete_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  ikd_init_ = false;
  fusion_node_num = 0;
  same_subgrid_edge_num = 0;
  trajectory_edd_num = 0;
  disconnected_nodes_ind_.clear();
}

//  添加函数 点  和  边 进行维护  的时候顺便  添加ikd树 节点
void MergerGraph::ProcessOneceData(
    std::unordered_map<int, tare_planner::msg::SharedMergerInfor>& Shared_Infor_map)
{
  ikd_add_cloud_->clear();
  for (auto& Shared_msg : Shared_Infor_map)
  {
    //  当前点属于  哪个   robot    id
    int robot_id = Shared_msg.first;
    //  新增点
    for (auto& node : Shared_msg.second.new_node_set)
    {
      //  新增点
      int keypose_node_ind_ = node.node_index;
      int new_node_ind = GetNewNodeIndex();  //新增点索引为  第一个现在现存的节点个数
      AddNode(node.node_position, new_node_ind, 0, node.is_keypose);
      //  设置   点  与   merger_graph  的索引转换    边关系 使用     索引连接
      robot_keyposeind_mergerind_[robot_id][keypose_node_ind_] = new_node_ind;
    }
    //  新增  边
    for (auto& add_edge : Shared_msg.second.add_edge_set)
    {
      int merger_ind_1 = robot_keyposeind_mergerind_[robot_id][add_edge.node_index_min];
      int merger_ind_2 = robot_keyposeind_mergerind_[robot_id][add_edge.node_index_max];
      // 新增边
      geometry_msgs::msg::Point node_1 = GetNodePosition(merger_ind_1);
      geometry_msgs::msg::Point node_2 = GetNodePosition(merger_ind_2);
      double dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(node_1, node_2);
      AddEdge(merger_ind_1, merger_ind_2, dist);
    }
    // 删除  边
    for (auto& delete_edge : Shared_msg.second.delete_edge_set)
    {
      int merger_ind_1 = robot_keyposeind_mergerind_[robot_id][delete_edge.node_index_min];
      int merger_ind_2 = robot_keyposeind_mergerind_[robot_id][delete_edge.node_index_max];
      DeleteEdge(merger_ind_1, merger_ind_2);
    }
  }
  //   累积  本次的     新增  点  更新到    ikd_add_cloud_
  tare_planner::msg::SharedMergerInfor temp = GetSharedInfor();
  for (auto& node : temp.new_node_set)
  {
    pcl::PointXYZI point;
    point.x = node.node_position.x;
    point.y = node.node_position.y;
    point.z = node.node_position.z;
    point.intensity = node.node_index;  //强度  代表  索引
    ikd_add_cloud_->push_back(point);
  }
  // 点云 建     ikd树  一颗  所有节点的树     一颗  可通行的节点树
  if (!ikd_init_)
  {
    if ((*ikd_add_cloud_).points.size() > 0)
    {
      // 使用实参  复制
      ikd_nodes_->Build((*ikd_add_cloud_).points);
      ikd_connected_nodes_->Build((*ikd_add_cloud_).points);
      ikd_init_ = true;
    }
  }
  else
  {
    // 每次添加
    ikd_nodes_->Add_Points((*ikd_add_cloud_).points, false);
    ikd_connected_nodes_->Add_Points((*ikd_add_cloud_).points, false);
  }
}

// void merger_graph_ns::MergerGraph::ProcessOneceDataAddEdge(
void MergerGraph::ProcessOneceDataAddEdge(
    std::unordered_map<int, tare_planner::msg::SharedMergerInfor>& Shared_Infor_map)
{
  // 清空单次节点
  ikd_add_cloud_->clear();
  for (auto& Shared_msg : Shared_Infor_map)
  {
    //  当前点属于  哪个   robot    id
    int robot_id = Shared_msg.first;
    //  新增  边
    for (auto& add_edge : Shared_msg.second.add_edge_set)
    {
      int merger_ind_1 = robot_keyposeind_mergerind_[robot_id][add_edge.node_index_min];
      int merger_ind_2 = robot_keyposeind_mergerind_[robot_id][add_edge.node_index_max];
      // 新增边
      geometry_msgs::msg::Point node_1 = GetNodePosition(merger_ind_1);
      geometry_msgs::msg::Point node_2 = GetNodePosition(merger_ind_2);
      double dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(node_1, node_2);
      AddEdge(merger_ind_1, merger_ind_2, dist);
    }
    // 删除  边
    for (auto& delete_edge : Shared_msg.second.delete_edge_set)
    {
      int merger_ind_1 = robot_keyposeind_mergerind_[robot_id][delete_edge.node_index_min];
      int merger_ind_2 = robot_keyposeind_mergerind_[robot_id][delete_edge.node_index_max];
      DeleteEdge(merger_ind_1, merger_ind_2);
    }
  }
  //   累积  本次的     新增  点  更新到    ikd_add_cloud_
  tare_planner::msg::SharedMergerInfor temp = GetSharedInfor();
  for (auto& node : temp.new_node_set)
  {
    pcl::PointXYZI point;
    point.x = node.node_position.x;
    point.y = node.node_position.y;
    point.z = node.node_position.z;
    point.intensity = node.node_index;  //强度  代表  索引
    ikd_add_cloud_->push_back(point);
  }
  // 点云 建     ikd树  一颗  所有节点的树     一颗  可通行的节点树
  if (!ikd_init_)
  {
    if ((*ikd_add_cloud_).points.size() > 0)
    {
      // 使用实参  复制
      ikd_nodes_->Build((*ikd_add_cloud_).points);
      ikd_connected_nodes_->Build((*ikd_add_cloud_).points);
      ikd_init_ = true;
    }
  }
  else
  {
    // 每次添加
    if ((*ikd_add_cloud_).points.size() > 0)
    {
      // 使用实参  复制   ikd 树维护   加点
      ikd_nodes_->Add_Points((*ikd_add_cloud_).points, false);
      ikd_connected_nodes_->Add_Points((*ikd_add_cloud_).points, false);
      // std::cout << "每次添加之后  树的节点个数" << std::endl;
      // std::cout << "ikd_nodes_  .size()  " << ikd_nodes_->validnum() << std::endl;
      // std::cout << "ikd_connected_nodes_  .size()  " << ikd_connected_nodes_->validnum() << std::endl;
    }
  }
}

//  获得不连通的节点  判定是否需要 减少    用于 维护   ikd  树  做  删除  与增加
void MergerGraph::MaintainConnectedNodesIkdtree()
{
  // 未初始化   不做处理
  if (!ikd_init_)
  {
    return;
  }
  //查询本次 不连通的节点索引
  std::vector<int> new_disconnected_nodes_ind = GetDisConnectedNodeIndes();
  // std::cout << "当前  不连接的点云数   " << new_disconnected_nodes_ind.size() << std::endl;
  // std::cout << "前一次    不连接的点云数   " << disconnected_nodes_ind_.size() << std::endl;
  // 求差
  std::vector<int> diff_nodes_ind_delete;  //删点  后面增加的点
  std::vector<int> diff_nodes_ind_add;     // 加点   之前删除之后 现在  添加上的点
  // 当前不连接  - 前一次不连接   则是  这次要删除 的
  misc_utils_ns::SetDifference(new_disconnected_nodes_ind, disconnected_nodes_ind_,
                               diff_nodes_ind_delete);  //得到不同的编号
  // std::cout << "本次要删除的   " << diff_nodes_ind_delete.size() << std::endl;
  //  上一次不  连接  -  本次不连接  则是  这次要添加的
  misc_utils_ns::SetDifference(disconnected_nodes_ind_, new_disconnected_nodes_ind,
                               diff_nodes_ind_add);  //得到不同的编号
  // std::cout << "本次要添加的   " << diff_nodes_ind_add.size() << std::endl;

  // std::cout << "维护 连通的  Ikd树" << std::endl;

  //  维护
  //  加点
  if (diff_nodes_ind_add.size() > 0)
  {
    pcl::PointCloud<pcl::PointXYZI> add_nodes;
    for (size_t i = 0; i < diff_nodes_ind_add.size(); i++)
    {
      pcl::PointXYZI point;
      geometry_msgs::msg::Point temp = GetNodePosition(diff_nodes_ind_add[i]);
      point.x = temp.x;
      point.y = temp.y;
      point.z = temp.z;
      point.intensity = diff_nodes_ind_add[i];
      add_nodes.push_back(point);
    }
    // std::cout << "加点 个数  =  " << add_nodes.points.size() << std::endl;
    ikd_connected_nodes_->Add_Points(add_nodes.points);
  }

  if (diff_nodes_ind_delete.size() > 0)
  {
    pcl::PointCloud<pcl::PointXYZI> delete_nodes;  //删除单是局部  信息
    for (int i = 0; i < diff_nodes_ind_delete.size(); i++)
    {
      pcl::PointXYZI point;
      geometry_msgs::msg::Point temp = GetNodePosition(diff_nodes_ind_delete[i]);
      point.x = temp.x;
      point.y = temp.y;
      point.z = temp.z;
      point.intensity = diff_nodes_ind_delete[i];
      delete_nodes.push_back(point);
    }
    ikd_connected_nodes_->Delete_Points(delete_nodes.points);
  }
  //   获得总的节点个数
  int node_all = GetNodeNum();
  int node_connected_num = node_all - new_disconnected_nodes_ind.size();

  // 更新本次不连通点索引
  disconnected_nodes_ind_ = new_disconnected_nodes_ind;
  // 删除本次    加点  加边的信息
  ClearSharedInfor();
}

//  重写   kd  相关  函数   使用  ikd  代替

// 得到 最近节点  和 距离  在所有节点上搜索
void MergerGraph::GetClosestNodeIndAndDistanceIkdtree(const geometry_msgs::msg::Point& point, int& node_ind, double& dist)
{
  node_ind = -1;
  dist = DBL_MAX;
  // 有效个数为0
  if (ikd_nodes_->validnum() == 0)
  {
    node_ind = -1;
    dist = DBL_MAX;
    return;
  }
  pcl::PointXYZI search_point;
  search_point.x = point.x;
  search_point.y = point.y;
  search_point.z = point.z;
  // std::vector<int> nearest_neighbor_node_indices(1);
  //
  pcl::PointCloud<pcl::PointXYZI> nearest_neighbor_node;
  std::vector<float> nearest_neighbor_squared_dist(1);
  ikd_nodes_->Nearest_Search(search_point, 1, nearest_neighbor_node.points, nearest_neighbor_squared_dist);
  // kdtree_nodes_->nearestKSearch(search_point, 1, nearest_neighbor_node_indices, nearest_neighbor_squared_dist);

  if (!nearest_neighbor_node.points.empty())
  {
    node_ind = static_cast<int>(nearest_neighbor_node.points.front().intensity);
    dist = sqrt(nearest_neighbor_squared_dist.front());
  }
  else
  {
    // ROS_WARN_STREAM("KeyposeGraph::GetClosestNodeIndAndDistance: search for nearest neighbor failed with "
    //                 << ikd_nodes_->validnum() << " nodes.");
    // if (!nearest_neighbor_node_indices.empty())
    // {
    //   ROS_WARN_STREAM("Nearest neighbor node Ind: " << nearest_neighbor_node_indices.front());
    // }
    int node_num = GetNewNodeIndex();
    for (int i = 0; i < node_num; i++)
    {
      geometry_msgs::msg::Point node_position = GetNodePosition(i);
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

// 得到  最近节点  和 距离 在  连接节点  上搜索
void MergerGraph::GetClosestConnectedNodeIndAndDistanceIkdtree(const geometry_msgs::msg::Point& point, int& node_ind,
                                                               double& dist)
{
  if (ikd_connected_nodes_->validnum() == 0)
  {
    node_ind = -1;
    dist = DBL_MAX;
    return;
  }
  pcl::PointXYZI search_point;
  search_point.x = point.x;
  search_point.y = point.y;
  search_point.z = point.z;
  // std::vector<int> nearest_neighbor_node_indices(1);
  pcl::PointCloud<pcl::PointXYZI> nearest_neighbor_node;
  std::vector<float> nearest_neighbor_squared_dist(1);
  // kdtree_connected_nodes_->nearestKSearch(search_point, 1,
  // nearest_neighbor_node_indices,nearest_neighbor_squared_dist);//  以 search_point  为基准点，取 1 个点  ，最近点的
  // nearest_neighbor_node_indices 下标，最近点的距离nearest_neighbor_squared_dist
  ikd_connected_nodes_->Nearest_Search(search_point, 1, nearest_neighbor_node.points, nearest_neighbor_squared_dist);
  if (!nearest_neighbor_node.points.empty())
  {
    node_ind = static_cast<int>(nearest_neighbor_node.points.front().intensity);
    dist = sqrt(nearest_neighbor_squared_dist.front());  //  开方？
  }
  else
  {
    // ROS_WARN_STREAM("KeyposeGraph::GetClosestNodeInd: search for nearest neighbor failed with "
    //                 << ikd_connected_nodes_->validnum() << " connected nodes.");
    RCLCPP_WARN_STREAM(node_->get_logger(), "KeyposeGraph::GetClosestNodeInd: search for nearest neighbor failed with " << ikd_connected_nodes_->validnum() << " connected nodes.");
    node_ind = -1;
    dist = 0;
  }
}

// 得到范围内的  连接节点
void MergerGraph::GetRangeClosestConnectedNodeIndIkdtree(const geometry_msgs::msg::Point& point, float range,
                                                         std::vector<int>& node_indices)
{
  if (ikd_connected_nodes_->validnum() == 0)
  {
    return;
  }
  pcl::PointXYZI search_point;
  search_point.x = point.x;
  search_point.y = point.y;
  search_point.z = point.z;
  // std::vector<int> nearest_neighbor_node_indices(1);
  pcl::PointCloud<pcl::PointXYZI> nearest_neighbor_node;
  std::vector<float> nearest_neighbor_squared_dist(1);
  // kdtree_connected_nodes_->nearestKSearch(search_point, 1,
  // nearest_neighbor_node_indices,nearest_neighbor_squared_dist);//  以 search_point  为基准点，取 1 个点  ，最近点的
  // nearest_neighbor_node_indices 下标，最近点的距离nearest_neighbor_squared_dist
  // ikd_connected_nodes_->Nearest_Search(search_point, 1, nearest_neighbor_node.points, nearest_neighbor_squared_dist);
  ikd_connected_nodes_->Radius_Search(search_point, range, nearest_neighbor_node.points);
  if (!nearest_neighbor_node.points.empty())
  {
    for (int i = 0; i < nearest_neighbor_node.points.size(); i++)
    {
      node_indices.push_back(nearest_neighbor_node.points[i].intensity);  //强度就是  索引  返回即可
    }
  }
  else
  {
    // ROS_WARN_STREAM("GetRangeClosestConnectedNodeIndIkdtree: search for nearest neighbor failed with "
    //                 << ikd_connected_nodes_->validnum() << " connected nodes.");
    RCLCPP_WARN_STREAM(node_->get_logger(), "GetRangeClosestConnectedNodeIndIkdtree: search for nearest neighbor failed with " << ikd_connected_nodes_->validnum() << " connected nodes.");
  }
}

//重写 HasNode  判定
bool MergerGraph::HasNodeIkdtree(const Eigen::Vector3d& position)
{
  int closest_node_ind = -1;
  double min_dist = DBL_MAX;
  geometry_msgs::msg::Point geo_position;
  geo_position.x = position.x();
  geo_position.y = position.y();
  geo_position.z = position.z();
  // 使用 Ikdtree 计算
  GetClosestNodeIndAndDistanceIkdtree(geo_position, closest_node_ind, min_dist);  //得到当前结点最近的结点距离
  int nodes_num = GetNodeNum();
  if (closest_node_ind >= 0 && closest_node_ind < nodes_num)
  {
    double xy_dist = misc_utils_ns::PointXYDist<geometry_msgs::msg::Point>(geo_position, GetNodePosition(closest_node_ind));
    double z_dist = std::abs(geo_position.z - GetNodePosition(closest_node_ind).z);
    if (xy_dist < SetAddNonKeyposeNodeMinDist() && z_dist < 1.0)
    {
      return true;
    }
  }
  return false;
}
//重写 两个位置 是否连接
bool MergerGraph::IsConnctedIkdtree(const Eigen::Vector3d& from_position, const Eigen::Vector3d& to_position)
{
  geometry_msgs::msg::Point from_node_position;
  from_node_position.x = from_position.x();
  from_node_position.y = from_position.y();
  from_node_position.z = from_position.z();
  int closest_from_node_ind = -1;
  double closest_from_node_dist = DBL_MAX;
  GetClosestNodeIndAndDistanceIkdtree(from_node_position, closest_from_node_ind, closest_from_node_dist);

  geometry_msgs::msg::Point to_node_position;
  to_node_position.x = to_position.x();
  to_node_position.y = to_position.y();
  to_node_position.z = to_position.z();
  int closest_to_node_ind = -1;
  double closest_to_node_dist = DBL_MAX;
  GetClosestNodeIndAndDistanceIkdtree(to_node_position, closest_to_node_ind, closest_to_node_dist);

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
// 重写  检查 连通性  设置 连通性
void MergerGraph::CheckConnectivityIkdtree(const geometry_msgs::msg::Point& robot_position)
{
  if (GetNodeNum() == 0)
  {
    return;
  }
  // UpdateNodes();     //更新结点    这是     所有节点的kd树相关的点云维护  使用  ikd 树 不需要维护

  // The first keypose node is always connected, set all the others to be disconnected      第一个关键位姿即诶单时相连的
  // ，设置其他的不连通
  int first_keypose_node_ind = 0;
  bool found_connected = false;
  int node_num = GetNodeNum();
  // for (int i = 0; i < node_num; i++)
  // {
  //   if (GetNodeIsKeypose(i))  //当找到是关键位姿的结点  就跳出循环    机器人行走过的位置肯定是连通的
  //   {
  //     first_keypose_node_ind = i;
  //     break;
  //   }
  // }

  first_keypose_node_ind = current_mergerpose_id_;

  // Check the connectivity starting from the robot    机器人开始前  检测是否连通
  for (int i = 0; i < node_num; i++)  //初始化  所有结点 都   不连通
  {
    // nodes_[i].is_connected_ = false;
    SetNodeIsConnected(i, false);  //初始化 所有为 false
  }
  std::vector<int> connected_node_indices;
  if (first_keypose_node_ind >= 0 && first_keypose_node_ind < node_num)  //存在关键位姿结点
  {
    // nodes_[first_keypose_node_ind].is_connected_ = true;  //第一个  是航迹点  肯定是连接的
    SetNodeIsConnected(first_keypose_node_ind, true);
    // connected_node_indices_.clear();//清空连接点集
    ClearConnectedNodeIndices();
    std::vector<bool> constraint(node_num, true);                                         //全部设置为  true
    GetConnectedNodeIndices(first_keypose_node_ind, connected_node_indices, constraint);  //得到连接的结点  ID
    SetConnectedNodeIndices(connected_node_indices);
    // std::cout << "connected_node_indices  = " << connected_node_indices.size() << std::endl;
  }
  else  //不存在关键位姿结点         即才初始化
  {
    int robot_node_ind = -1;
    double robot_node_dist = DBL_MAX;
    GetClosestNodeIndAndDistanceIkdtree(robot_position, robot_node_ind,
                                        robot_node_dist);  //得到距离  机器人位置     最近结点   的关键位姿图上的 结点id
                                                           //和距离
    if (robot_node_ind >= 0 && robot_node_ind < node_num)
    {
      // nodes_[robot_node_ind].is_connected_ = true;
      SetNodeIsConnected(robot_node_ind, true);
      // connected_node_indices_.clear();
      ClearConnectedNodeIndices();
      std::vector<bool> constraint(node_num, true);
      GetConnectedNodeIndices(robot_node_ind, connected_node_indices, constraint);  //得到与  robot_node_ind  相连的  ID
      SetConnectedNodeIndices(connected_node_indices);
      // std::cout << "connected_node_indices  = " << connected_node_indices.size() << std::endl;
    }
    else
    {
    //   ROS_ERROR_STREAM("KeyposeGraph::CheckConnectivity: Cannot get closest robot node ind "
    //                    << robot_node_ind);  //不能得到最近的机器人结点 ID
      RCLCPP_ERROR_STREAM(node_->get_logger(), "KeyposeGraph::CheckConnectivity: Cannot get closest robot node ind " << robot_node_ind);
    }
  }
  //  将  连通的设置为 ture
  SetConnectedNodeIndices();
  //  为  kd 树  维护   连通点云的数据connected_nodes_cloud_   使用 Ikdtree  不需要  屏蔽
  // 更换为  Ikd树  的  加点  减点
  //
  MaintainConnectedNodesIkdtree();
  //打印  累积到现在  融合点  添加边   个数
  if (false)
  {
    std::cout << "-------------------拼接图融合信息---------------     " << std::endl;
    std::cout << "融合节点                  fusion_node_num                    =   " << fusion_node_num << std::endl;
    std::cout << "同子网格添加边数 same_subgrid_edge_num    =   " << same_subgrid_edge_num << std::endl;
    std::cout << "轨迹边添加数          trajectory_edd_num                =   " << trajectory_edd_num << std::endl;
  }
}

// 重写 最近节点  index
int MergerGraph::GetClosestNodeIndIkdtree(const geometry_msgs::msg::Point& point)
{
  int node_ind = 0;
  double min_dist = DBL_MAX;
  GetClosestNodeIndAndDistanceIkdtree(point, node_ind, min_dist);
  return node_ind;
}

// 重写  位置是否可以到达  两个参数   即 在   显示给的范围 dist_threshold    内  认为是能够到达的
bool MergerGraph::IsPositionReachableIkdtree(const geometry_msgs::msg::Point& point, double dist_threshold)
{
  int closest_node_ind = 0;
  double closest_node_dist = DBL_MAX;
  GetClosestConnectedNodeIndAndDistanceIkdtree(point, closest_node_ind, closest_node_dist);
  int node_num = GetNodeNum();
  if (closest_node_ind >= 0 && closest_node_ind < node_num && closest_node_dist < dist_threshold &&
      GetNodeIsConnected(closest_node_ind))
  {
    return true;
  }
  else
  {
    return false;
  }
}
// 重写  位置是否可以到达  单参数  使用  默认参数
bool MergerGraph::IsPositionReachableIkdtree(const geometry_msgs::msg::Point& point)
{
  int closest_node_ind = 0;
  double closest_node_dist = DBL_MAX;
  GetClosestConnectedNodeIndAndDistanceIkdtree(point, closest_node_ind, closest_node_dist);
  int node_num = GetNodeNum();
  if (closest_node_ind >= 0 && closest_node_ind < node_num && closest_node_dist < SetAddNonKeyposeNodeMinDist() &&
      GetNodeIsConnected(closest_node_ind))  //   小于0.5认为是可达到
  {
    return true;
  }
  else
  {
    return false;
  }
}
// 重写  求最短路径
bool MergerGraph::GetShortestPathTwoPointIkdtree(const geometry_msgs::msg::Point& start_point,
                                                 const geometry_msgs::msg::Point& target_point, bool get_path,
                                                 nav_msgs::msg::Path& path, bool use_connected_nodes, double& kcellsize)
{
  int node_num = GetNodeNum();
  if (node_num < 2)
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
  // 使用  kd 数查找最近点  输出
  GetClosestConnectedNodeIndAndDistanceIkdtree(
      start_point, start_closest_node_ind, start_closest_node_dist);  //在关键位姿图上找到当前机器人位置最近的结点和距离
  // start_closest_node_dist  是在  keyposegrapher 中  可通行的  距离start_point  最近的点  距离为
  // start_closest_node_dist
  if (start_closest_node_dist < kcellsize && start_closest_node_ind >= 0 &&
      start_closest_node_ind < GetNodeNum())  //当找到的结点最小距离小于kCellSize/2 且结点的范围正常
  {
    start_point_ = GetNodePosition(start_closest_node_ind);  //当前机器人位置就是找到最近结点在关键位姿图上的位置
    flag_start = true;
    std::cout << "存在 start_point_ " << std::endl;
  }
  GetClosestConnectedNodeIndAndDistanceIkdtree(
      target_point, goal_closest_node_ind, goal_closest_node_dist);  //在关键位姿图上找到当前机器人位置最近的结点和距离
  if (goal_closest_node_dist < kcellsize && goal_closest_node_ind >= 0 &&
      goal_closest_node_ind < GetNodeNum())  //当找到的结点最小距离小于kCellSize/2 且结点的范围正常
  {
    goal_point_ = GetNodePosition(goal_closest_node_ind);  //当前机器人位置就是找到最近结点在关键位姿图上的位置
    flag_goal = true;
    std::cout << "存在 goal_point_ " << std::endl;
  }
  //  两个点不存在就   返回false
  if (!flag_goal || !flag_start)
  {
    return false;
  }
  //计算路径
  std::vector<geometry_msgs::msg::Point> node_positions;
  for (int i = 0; i < node_num; i++)
  {
    node_positions.push_back(GetNodePosition(i));
  }
  std::vector<int> path_indices;
  double shortest_dist = misc_utils_ns::AStarSearch(GetGraph(), GetDist(), node_positions, start_closest_node_ind,
                                                    goal_closest_node_ind, get_path, path_indices);
  if (path_indices.size() < 2)
  {
    std::cout << "路径数小于2  出错" << std::endl;
    return false;
  }
  if (get_path)  //如果存在路径   不存在路径就不返回
  {
    path.poses.clear();
    for (const auto& ind : path_indices)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position = GetNodePosition(ind);
      pose.pose.orientation.w = GetNodeIsKeypose(ind);  //路径的  旋转属性    w 为是否是  轨迹的标志
      pose.pose.orientation.x = ind;                    // 旋转的 x  的属性 为   编号
      path.poses.push_back(pose);
    }
  }
  return true;  //查找成功
}

// 新增  添加  边      查找当前位置  和  其他点位置的连通属性
//  在规划框范围内   获得当前小车的  连通  节点   索引     主动连接 其他  机器人的   节点
// 添加与当前  机器人  在范围   x  中的  其他   其他机器人  的点  的  连线  。
// 使用局部规划框  做  当前位置  和    其他机器人    节点 的   碰撞检测  连线  在  显示  范围  x 内
// 第一个参数是  机器人 位置  第二 个参数  考虑范围  第三个 参数 是局部规划 类
// int range = pd_.keypose_graph_->SetAddEdgeCollisionCheckRadius();
void MergerGraph::AddEdgeLocalRobotWithOtherRobotNodesInRange(
    const geometry_msgs::msg::Point& start_point_, float range, int robot_id, int cur_keypose_ind,
    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
    std::vector<std::pair<int, int>>& add_edge)
{
  //获取当前位置     点   的  索引
  // int robot_index_on_merger_graph=-1;
  // double dist;
  // GetClosestConnectedNodeIndAndDistanceIkdtree(start_point, robot_index_on_merger_graph, dist);
  // if(robot_index_on_merger_graph==-1)
  // {
  //   return;
  // }
  current_mergerpose_id_ = robot_keyposeind_mergerind_[robot_id][cur_keypose_ind];
  geometry_msgs::msg::Point start_point = GetNodePosition(current_mergerpose_id_);

  //  使用ikdtree    在  以   start_point     为中心 求  范围  range  的  节点
  std::vector<int> node_indices;
  GetRangeClosestConnectedNodeIndIkdtree(start_point, range, node_indices);

  // 选择  非本地机器人 的   节点
  std::vector<int> diff_robot_id_node_indices;
  for (auto& index : node_indices)
  {
    if (GetNodeRobotId(index) != robot_id)  //不是同一个机器人的点加入
    {
      diff_robot_id_node_indices.push_back(index);
    }
  }
  // 对所有非 机器人点
  for (auto& index : diff_robot_id_node_indices)
  {
    geometry_msgs::msg::Point end_point = GetNodePosition(index);
    //  检测高度约束
    // 判定  边的高度  高度  大于  1.5  则不连线
    double z_diff_to_target = std::abs(start_point.z - end_point.z);
    if (z_diff_to_target > 1.5)  //高度约束  不能飞   高度约束   考虑楼道上下层 和坡道  上下高度 约束
                                 //需要根据不同实际场景 调试  1.5 这个参数在目前 应该是够用的
    {
      continue;
    }

    //使用  局部规划 做碰撞检测  并连线
    Eigen::Vector3d viewpoint_resolution = viewpoint_manager->GetResolution();  //视点分辨率
    double collision_check_resolution =
        std::min(viewpoint_resolution.x(), viewpoint_resolution.y()) / 2;  // 需要除以2 么？
    Eigen::Vector3d start_position = Eigen::Vector3d(start_point.x, start_point.y, start_point.z);
    ;
    Eigen::Vector3d end_position = Eigen::Vector3d(end_point.x, end_point.y, end_point.z);
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
    //  连线  不同机器人的边      成功则    对   node_index_1   node_index_2  添加边  并添加标记
    if (is_connected && HasEdgeBetween(current_mergerpose_id_, index) == false)
    {
      // 添加边   添加标记
      double temp_dist = misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(start_point, end_point);
      AddEdge(current_mergerpose_id_, index, temp_dist);
      add_edge.push_back(std::pair<int, int>(current_mergerpose_id_, index));
      //添加  轨迹  其他机器人连线    边  个数
      Addtrajectory_edd_num();
    }
  }

  // 求当前路径  和节点 直线  的碰撞检测     通过 则   加边。
}

//  转换数据
void MergerGraph::UpdataOnceMergerGraphEdge(tare_planner::msg::MergerGraphEdge& shared_merger_graph_edge,
                                            const std::vector<std::pair<int, int>>& add_edge,
                                            const std::vector<std::pair<int, int>>& delete_edge)
{
  //  通过  mergerind_robot_keyposeind_  反向转换

  //  add
  for (auto& temp_edge : add_edge)
  {
    // 单个  节点  可以对应不  同的 机器人  节点所以要用   循环获取
    // 边的 索引 应该都是  能够找到的   防止  意外 添加  判定
    //  debug
    if (!(InBound(temp_edge.first) && InBound(temp_edge.second)))  //  当  其中  一个边 不在里面  不做处理
    {
      std::cout << "边节点出错！！！！！！" << std::endl;
      continue;
    }

    for (auto& node_map_1 : mergerind_robot_keyposeind_[temp_edge.first])
    {
      int robot_id_1 = node_map_1.first;
      int keypose_ind_1 = node_map_1.second;
      for (auto& node_map_2 : mergerind_robot_keyposeind_[temp_edge.second])
      {
        int robot_id_2 = node_map_2.first;
        int keypose_ind_2 = node_map_2.second;
        // 添加边  到  shared_merger_graph_edge
        tare_planner::msg::MergerEdge merger_edge;
        merger_edge.node_min_robot_id = robot_id_1;
        merger_edge.node_index_min = keypose_ind_1;
        merger_edge.node_max_robot_id = robot_id_2;
        merger_edge.node_index_max = keypose_ind_2;
        shared_merger_graph_edge.add_edge_set.push_back(merger_edge);
      }
    }
  }

  //  delete

  for (auto& temp_edge : delete_edge)
  {
    // 单个  节点  可以对应不  同的 机器人  节点所以要用   循环获取
    // 边的 索引 应该都是  能够找到的   防止  意外 添加  判定
    //  debug
    if (!(InBound(temp_edge.first) && InBound(temp_edge.second)))  //  当  其中  一个边 不在里面  不做处理
    {
      std::cout << "边节点出错！！！！！！" << std::endl;
      continue;
    }

    for (auto& node_map_1 : mergerind_robot_keyposeind_[temp_edge.first])
    {
      int robot_id_1 = node_map_1.first;
      int keypose_ind_1 = node_map_1.second;
      for (auto& node_map_2 : mergerind_robot_keyposeind_[temp_edge.second])
      {
        int robot_id_2 = node_map_2.first;
        int keypose_ind_2 = node_map_2.second;
        // 添加边  到  shared_merger_graph_edge
        tare_planner::msg::MergerEdge merger_edge;
        merger_edge.node_min_robot_id = robot_id_1;
        merger_edge.node_index_min = keypose_ind_1;
        merger_edge.node_max_robot_id = robot_id_2;
        merger_edge.node_index_max = keypose_ind_2;
        shared_merger_graph_edge.delete_edge_set.push_back(merger_edge);
      }
    }
  }
}

// 通过  索引  修改
void MergerGraph::UpdataOnceMergerGraphEdgeFromOthers(
    tare_planner::msg::MergerGraphEdge& MergerGraphEdge_Once_sub)
{
  // 添加边  先验证   边 对应的  端点  是否  存在     再  验证是否存在边     robot_keyposeind_mergerind_
  for (auto& temp_edge : MergerGraphEdge_Once_sub.add_edge_set)
  {
    if (robot_keyposeind_mergerind_.count(temp_edge.node_min_robot_id) &&
        robot_keyposeind_mergerind_[temp_edge.node_min_robot_id].count(temp_edge.node_index_min) &&
        robot_keyposeind_mergerind_.count(temp_edge.node_max_robot_id) &&
        robot_keyposeind_mergerind_[temp_edge.node_max_robot_id].count(temp_edge.node_index_max))  // 修改
    {
      int node_ind_1 = robot_keyposeind_mergerind_[temp_edge.node_min_robot_id][temp_edge.node_index_min];
      int node_ind_2 = robot_keyposeind_mergerind_[temp_edge.node_max_robot_id][temp_edge.node_index_max];
      // 查询  边是否存在    边不存在   添加边
      if (!HasEdgeBetween(node_ind_1, node_ind_2))
      {
        double temp_dist = misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
            GetNodePosition(node_ind_1), GetNodePosition(node_ind_2));
        AddEdge(node_ind_1, node_ind_2, temp_dist);
      }
    }
  }

  // 删除边
  for (auto& temp_edge : MergerGraphEdge_Once_sub.delete_edge_set)
  {
    if (robot_keyposeind_mergerind_.count(temp_edge.node_min_robot_id) &&
        robot_keyposeind_mergerind_[temp_edge.node_min_robot_id].count(temp_edge.node_index_min) &&
        robot_keyposeind_mergerind_.count(temp_edge.node_max_robot_id) &&
        robot_keyposeind_mergerind_[temp_edge.node_max_robot_id].count(temp_edge.node_index_max))  // 修改
    {
      int node_ind_1 = robot_keyposeind_mergerind_[temp_edge.node_min_robot_id][temp_edge.node_index_min];
      int node_ind_2 = robot_keyposeind_mergerind_[temp_edge.node_max_robot_id][temp_edge.node_index_max];
      // 查询    边是否存在    存在   则删除边
      if (HasEdgeBetween(node_ind_1, node_ind_2))
      {
        DeleteEdge(node_ind_1, node_ind_2);
      }
    }
  }
}

}  // namespace merger_graph_ns
