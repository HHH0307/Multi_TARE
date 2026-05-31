/**
 * @file local_coverage_planner.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that ensures coverage in the surroundings of the robot
 * @version 0.1
 * @date 2021-05-31
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "local_coverage_planner/local_coverage_planner.h"

namespace local_coverage_planner_ns {
const std::string LocalCoveragePlanner::kRuntimeUnit = "us";

bool LocalCoveragePlannerParameter::ReadParameters(rclcpp::Node::SharedPtr nh) {
  nh->get_parameter("kMinAddPointNumSmall", kMinAddPointNum);
  nh->get_parameter("kMinAddFrontierPointNum", kMinAddFrontierPointNum);
  nh->get_parameter("kGreedyViewPointSampleRange", kGreedyViewPointSampleRange);
  nh->get_parameter("kLocalPathOptimizationItrMax", kLocalPathOptimizationItrMax);

  return true;
}
LocalCoveragePlanner::LocalCoveragePlanner(rclcpp::Node::SharedPtr nh) : lookahead_point_update_(false), use_frontier_(true), local_coverage_complete_(false) {
  parameters_.ReadParameters(nh);
  is_exploring_ = true; // 新增
  navigation_viewpoint_num_ = 0; // 新增
}

int LocalCoveragePlanner::GetBoundaryViewpointIndex(const exploration_path_ns::ExplorationPath &global_path) {
  int boundary_viewpoint_index = robot_viewpoint_ind_;
  if (!global_path.nodes_.empty()) {
    if (viewpoint_manager_->InLocalPlanningHorizon(
            global_path.nodes_.front().position_)) {
      for (int i = 0; i < global_path.nodes_.size(); i++) {
        if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
            global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ||
            !viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_[i].position_)) 
        {
          break;
        }
        boundary_viewpoint_index = viewpoint_manager_->GetNearestCandidateViewPointInd(global_path.nodes_[i].position_);
      }
    }
  }
  return boundary_viewpoint_index;
}

//新增改动
int LocalCoveragePlanner::GetBoundaryViewpointIndices_opt(exploration_path_ns::ExplorationPath near_localcoverage_subgrid_path)
{
  return GetBoundaryViewpointIndex(near_localcoverage_subgrid_path);
}

//新增  全局局部路径 导航点
int LocalCoveragePlanner::GetBoundaryViewpointIndices_far(const exploration_path_ns::ExplorationPath& global_path)
{
   int boundary_viewpoint_index = robot_viewpoint_ind_;  //机器人当前视点
  if (!global_path.nodes_.empty())                      //存在全局 路径点
  {
    if (viewpoint_manager_->InLocalPlanningHorizon(
            global_path.nodes_.front().position_))  //第一个点在  局部规划范围      第一个点是机器人位置  是存在的
    {
      for (int i = 1; i < global_path.nodes_.size(); i++)  //对每个全局规划点
      {
        // 对于  全局导航路径  只有  
        if(viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_[i].position_))
        {
          boundary_viewpoint_index = viewpoint_manager_->GetNearestCandidateViewPointInd(global_path.nodes_[i].position_);
        }
        if((global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT || global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ) && viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_[i].position_))
        {
          boundary_viewpoint_index = viewpoint_manager_->GetNearestCandidateViewPointInd(global_path.nodes_[i].position_);
          break;
        }
        if(!viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_[i].position_))
        {
          break;
        }
      }
    }
  }
  return boundary_viewpoint_index;
}

// 新增  修改 添加邻近全局探索子网格  路径  导航点
void LocalCoveragePlanner::GetNavigationViewPointIndices_opt(std::vector<exploration_path_ns::ExplorationPath> near_localcoverage_subgrid_paths,std::vector<int>& navigation_viewpoint_indices)
{
  robot_viewpoint_ind_ = viewpoint_manager_->GetNearestCandidateViewPointInd(robot_position_);  //机器人最近的 视点  ID
  lookahead_viewpoint_ind_ =
      viewpoint_manager_->GetNearestCandidateViewPointInd(lookahead_point_);  //前视点  最近 的视点 ID    目标点
  if (!lookahead_point_update_ ||
      !viewpoint_manager_->InRange(lookahead_viewpoint_ind_))  //前视点  没有更新  或者  不在范围内
  {
    lookahead_viewpoint_ind_ = robot_viewpoint_ind_;  //前视点  （目标点）  设置为机器人位置
  }
  start_viewpoint_ind_ = robot_viewpoint_ind_;
  end_viewpoint_ind_ = robot_viewpoint_ind_;
  navigation_viewpoint_indices.push_back(robot_viewpoint_ind_);      //机器人位置
  navigation_viewpoint_indices.push_back(lookahead_viewpoint_ind_);  //前视点
  
  // 添加  邻近全局探索子网格  路径导航点 索引

  for (int i = 0; i < near_localcoverage_subgrid_paths.size(); i++)
  {
    int near_localcoverage_subgrid_viewpoint_ind = GetBoundaryViewpointIndices_opt(near_localcoverage_subgrid_paths[i]);
    if (i == 0)
    {
      start_viewpoint_ind_ = near_localcoverage_subgrid_viewpoint_ind;
    }
    else
    {
      end_viewpoint_ind_ = near_localcoverage_subgrid_viewpoint_ind;
    }
  }
  if (near_localcoverage_subgrid_paths.size() == 1)
  {
    end_viewpoint_ind_ = start_viewpoint_ind_;
  }
  navigation_viewpoint_indices.push_back(start_viewpoint_ind_);
  navigation_viewpoint_indices.push_back(end_viewpoint_ind_);
}

void LocalCoveragePlanner::UpdateViewPointCoveredPoint(
    std::vector<bool> &point_list, int viewpoint_index, bool use_array_ind) {
  for (const auto &point_ind : viewpoint_manager_->GetViewPointCoveredPointList(
           viewpoint_index, use_array_ind)) {
    MY_ASSERT(misc_utils_ns::InRange<bool>(point_list, point_ind));
    point_list[point_ind] = true;
  }
}
void LocalCoveragePlanner::UpdateViewPointCoveredFrontierPoint(
    std::vector<bool> &frontier_point_list, int viewpoint_index,
    bool use_array_ind) {
  for (const auto &point_ind :
       viewpoint_manager_->GetViewPointCoveredFrontierPointList(
           viewpoint_index, use_array_ind)) {
    MY_ASSERT(misc_utils_ns::InRange<bool>(frontier_point_list, point_ind));
    frontier_point_list[point_ind] = true;
  }
}

void LocalCoveragePlanner::EnqueueViewpointCandidates(
    std::vector<std::pair<int, int>> &cover_point_queue,
    std::vector<std::pair<int, int>> &frontier_queue,
    const std::vector<bool> &covered_point_list,
    const std::vector<bool> &covered_frontier_point_list,
    const std::vector<int> &selected_viewpoint_array_indices) {
  for (const auto &viewpoint_index :
       viewpoint_manager_->GetViewPointCandidateIndices()) {
    if (viewpoint_manager_->ViewPointVisited(viewpoint_index) ||
        !viewpoint_manager_->ViewPointInExploringCell(viewpoint_index)) {
      continue;
    }
    int viewpoint_array_index =
        viewpoint_manager_->GetViewPointArrayInd(viewpoint_index);
    if (std::find(selected_viewpoint_array_indices.begin(),
                  selected_viewpoint_array_indices.end(),
                  viewpoint_array_index) !=
        selected_viewpoint_array_indices.end()) {
      continue;
    }
    int covered_point_num = viewpoint_manager_->GetViewPointCoveredPointNum(
        covered_point_list, viewpoint_array_index, true);
    if (covered_point_num >= parameters_.kMinAddPointNum) {
      cover_point_queue.emplace_back(covered_point_num, viewpoint_index);
    } else if (use_frontier_) {
      int covered_frontier_point_num =
          viewpoint_manager_->GetViewPointCoveredFrontierPointNum(
              covered_frontier_point_list, viewpoint_array_index, true);
      if (covered_frontier_point_num >= parameters_.kMinAddFrontierPointNum) {
        frontier_queue.emplace_back(covered_frontier_point_num,
                                    viewpoint_index);
      }
    }
  }

  // Sort the queue
  std::sort(cover_point_queue.begin(), cover_point_queue.end(), SortPairInRev);
  if (use_frontier_) {
    std::sort(frontier_queue.begin(), frontier_queue.end(), SortPairInRev);
  }
}

void LocalCoveragePlanner::SelectViewPoint(
    const std::vector<std::pair<int, int>> &queue,
    const std::vector<bool> &covered,
    std::vector<int> &selected_viewpoint_indices, bool use_frontier) {
  if (use_frontier) {
    if (queue.empty() || queue[0].first < parameters_.kMinAddFrontierPointNum) {
      return;
    }
  } else {
    if (queue.empty() || queue[0].first < parameters_.kMinAddPointNum) {
      return;
    }
  }

  std::vector<bool> covered_copy;
  for (int i = 0; i < covered.size(); i++) {
    covered_copy.push_back(covered[i]);
  }
  std::vector<std::pair<int, int>> queue_copy;
  for (int i = 0; i < queue.size(); i++) {
    queue_copy.push_back(queue[i]);
  }

  int sample_range = 0;
  for (int i = 0; i < queue_copy.size(); i++) {
    if (use_frontier) {
      if (queue_copy[i].first >= parameters_.kMinAddFrontierPointNum) {
        sample_range++;
      }
    } else {
      if (queue_copy[i].first >= parameters_.kMinAddPointNum) {
        sample_range++;
      }
    }
  }

  sample_range = std::min(parameters_.kGreedyViewPointSampleRange, sample_range);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> gen_next_queue_idx(0, sample_range - 1);
  int queue_idx = gen_next_queue_idx(gen);
  int cur_ind = queue_copy[queue_idx].second;

  while (true) {
    int cur_array_ind = viewpoint_manager_->GetViewPointArrayInd(cur_ind);
    if (use_frontier) {
      for (const auto &point_ind :
           viewpoint_manager_->GetViewPointCoveredFrontierPointList(
               cur_array_ind, true))

      {
        MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));
        if (!covered_copy[point_ind]) {
          covered_copy[point_ind] = true;
        }
      }
    } else {
      for (const auto &point_ind :
           viewpoint_manager_->GetViewPointCoveredPointList(cur_array_ind,
                                                            true)) {
        MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));
        if (!covered_copy[point_ind]) {
          covered_copy[point_ind] = true;
        }
      }
    }
    selected_viewpoint_indices.push_back(cur_ind);
    queue_copy.erase(queue_copy.begin() + queue_idx);

    // Update the queue
    for (int i = 0; i < queue_copy.size(); i++) {
      int add_point_num = 0;
      int ind = queue_copy[i].second;
      int array_ind = viewpoint_manager_->GetViewPointArrayInd(ind);
      if (use_frontier) {
        for (const auto &point_ind :
             viewpoint_manager_->GetViewPointCoveredFrontierPointList(array_ind,
                                                                      true)) {
          MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));
          if (!covered_copy[point_ind]) {
            add_point_num++;
          }
        }
      } else {
        for (const auto &point_ind :
             viewpoint_manager_->GetViewPointCoveredPointList(array_ind,
                                                              true)) {
          MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));
          if (!covered_copy[point_ind]) {
            add_point_num++;
          }
        }
      }

      queue_copy[i].first = add_point_num;
    }

    std::sort(queue_copy.begin(), queue_copy.end(), SortPairInRev);

    if (queue_copy.empty() || queue_copy[0].first < parameters_.kMinAddPointNum) {
      break;
    }
    if (use_frontier) {
      if (queue_copy.empty() || queue_copy[0].first < parameters_.kMinAddFrontierPointNum) {
        break;
      }
    }

    // Randomly select the next point
    int sample_range = 0;
    for (int i = 0; i < queue.size(); i++) {
      if (use_frontier) {
        if (queue[i].first >= parameters_.kMinAddFrontierPointNum) {
          sample_range++;
        }
      } else {
        if (queue[i].first >= parameters_.kMinAddPointNum) {
          sample_range++;
        }
      }
    }
    sample_range =
        std::min(parameters_.kGreedyViewPointSampleRange, sample_range);
    std::uniform_int_distribution<int> gen_next_queue_idx(0, sample_range - 1);
    queue_idx = gen_next_queue_idx(gen);
    cur_ind = queue_copy[queue_idx].second;
  }
}

//  修改随机选择，直接从   感知最大的sample_range  中  选择   距离障碍物最远的点    不用随机数，而是考虑远离障碍物
void LocalCoveragePlanner::SelectViewPoint_UseDistance(const std::vector<std::pair<int, int>>& queue,
                                                       const std::vector<bool>& covered,
                                                       std::vector<int>& selected_viewpoint_indices, bool use_frontier)
{
  //  use_frontier  不同   使用的  参数不同   逻辑是一样的
  if (use_frontier)
  {
    if (queue.empty() || queue[0].first < parameters_.kMinAddFrontierPointNum)
    {
      return;
    }
  }
  else
  {
    if (queue.empty() || queue[0].first < parameters_.kMinAddPointNum)
    {
      return;
    }
  }

  std::vector<bool> covered_copy;
  for (int i = 0; i < covered.size(); i++)
  {
    covered_copy.push_back(covered[i]);
  }
  std::vector<std::pair<int, int>> queue_copy;
  for (int i = 0; i < queue.size(); i++)
  {
    queue_copy.push_back(queue[i]);
  }

  //  判定  某点  的增益（感知）属性   >   外部传入的阈值参数  作为参与采样的个数
  int sample_range = 0;
  for (int i = 0; i < queue_copy.size(); i++)
  {
    if (use_frontier)
    {
      if (queue_copy[i].first >= parameters_.kMinAddFrontierPointNum)  //  判定  某点  的增益（感知）属性   >
                                                                       //  外部传入的阈值参数  作为参与采样的个数
      {
        sample_range++;
      }
    }
    else
    {
      if (queue_copy[i].first >= parameters_.kMinAddPointNum)
      {
        sample_range++;
      }
    }
  }

  sample_range = std::min(parameters_.kGreedyViewPointSampleRange, sample_range); 
  double max_dist = 0;
  int cur_ind = queue_copy[0].second;
  int queue_idx = 0;
  for (int i = 0; i < sample_range; i++)
  {
    int temp_ind = queue_copy[i].second;
    int temp_array_ind = viewpoint_manager_->GetViewPointArrayInd(temp_ind);
    double temp_dist = viewpoint_manager_->GetLidarNearestObstacleDistance(temp_array_ind, true);
    // std::cout << " 距离: " << temp_dist << std::endl;
    if (temp_dist > max_dist)
    {
      cur_ind = temp_ind;
      max_dist = temp_dist;
      queue_idx = i;
    }
  }
  // std::cout << " 最大距离:   " << max_dist << std::endl;

  //核心
  while (true)
  {
    //选中  cur_ind
    int cur_array_ind = viewpoint_manager_->GetViewPointArrayInd(cur_ind);
    //计算 cur_ind   这个点  覆盖了 多少的点  并标记那些点为已覆盖
    if (use_frontier)
    {
      for (const auto& point_ind : viewpoint_manager_->GetViewPointCoveredFrontierPointList(cur_array_ind, true))

      {
        MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));
        if (!covered_copy[point_ind])
        {
          covered_copy[point_ind] = true;  // 标记已覆盖
        }
      }
    }
    else
    {
      for (const auto& point_ind : viewpoint_manager_->GetViewPointCoveredPointList(cur_array_ind, true))
      {
        MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));
        if (!covered_copy[point_ind])
        {
          covered_copy[point_ind] = true;  // 标记已覆盖
        }
      }
    }
    //选中点  加入返回变量中
    selected_viewpoint_indices.push_back(cur_ind);
    //在候选队列中删除 选中的点
    queue_copy.erase(queue_copy.begin() + queue_idx);

    // Update the queue
    //  更新候选队列中      每个点的未覆盖值（更新奖励）
    for (int i = 0; i < queue_copy.size(); i++)
    {
      int add_point_num = 0;
      int ind = queue_copy[i].second;
      int array_ind = viewpoint_manager_->GetViewPointArrayInd(ind);
      if (use_frontier)
      {
        for (const auto& point_ind : viewpoint_manager_->GetViewPointCoveredFrontierPointList(array_ind, true))
        {
          MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));
          if (!covered_copy[point_ind])  //标记 没覆盖的点
          {
            add_point_num++;  //奖励+1
          }
        }
      }
      else
      {
        for (const auto& point_ind : viewpoint_manager_->GetViewPointCoveredPointList(array_ind, true))
        {
          MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));
          if (!covered_copy[point_ind])
          {
            add_point_num++;
          }
        }
      }

      queue_copy[i].first = add_point_num;  //将没覆盖的奖励更新到队列中
    }

    std::sort(queue_copy.begin(), queue_copy.end(), SortPairInRev);  //重排序
                               // 判断跳出循环    点被选完            或者     候选的点  最大的奖励 也小于    某个参数
                               // parameters_.kMinAddPointNum
    if (queue_copy.empty() || queue_copy[0].first < parameters_.kMinAddPointNum)
    {
      break;
    }
    if (use_frontier)
    {
      if (queue_copy.empty() || queue_copy[0].first < parameters_.kMinAddFrontierPointNum)
      {
        break;
      }
    }

    // Randomly select the next point
    // 再次随机选一个点queue_copy[i].first
    int sample_range = 0;
    for (int i = 0; i < queue_copy.size(); i++)
    {
      if (use_frontier)
      {
        if (queue_copy[i].first >= parameters_.kMinAddFrontierPointNum)  //当 奖励     大于  最小阈值
                                                                         //则将当前点加入随机获取的队列中
        {
          sample_range++;
        }
      }
      else
      {
        if (queue_copy[i].first >= parameters_.kMinAddPointNum)  // 当 奖励     大于  最小阈值
                                                                 // 则将当前点加入随机获取的队列中
        {
          sample_range++;
        }
      }
    }

    sample_range = std::min(parameters_.kGreedyViewPointSampleRange, sample_range); 
    max_dist = -1;
    cur_ind = queue_copy[0].second;
    queue_idx = 0;
    for (int i = 0; i < sample_range; i++)
    {
      int temp_ind = queue_copy[i].second;
      int temp_array_ind = viewpoint_manager_->GetViewPointArrayInd(temp_ind);
      double temp_dist = viewpoint_manager_->GetLidarNearestObstacleDistance(temp_array_ind, true);
      if (temp_dist > max_dist)
      {
        cur_ind = temp_ind;
        max_dist = temp_dist;
        queue_idx = i;
      }
    }
  }
}

void LocalCoveragePlanner::SelectViewPointFromFrontierQueue(
    std::vector<std::pair<int, int>> &frontier_queue,
    std::vector<bool> &frontier_covered,
    std::vector<int> &selected_viewpoint_indices) {
  if (use_frontier_ && !frontier_queue.empty() && frontier_queue[0].first > parameters_.kMinAddFrontierPointNum) {
    // Update the frontier queue
    for (const auto &ind : selected_viewpoint_indices) {
      UpdateViewPointCoveredFrontierPoint(frontier_covered, ind);
    }
    for (int i = 0; i < frontier_queue.size(); i++) {
      int ind = frontier_queue[i].second;
      int covered_frontier_point_num = viewpoint_manager_->GetViewPointCoveredFrontierPointNum(frontier_covered, ind);
      frontier_queue[i].first = covered_frontier_point_num;
    }
    std::sort(frontier_queue.begin(), frontier_queue.end(), SortPairInRev);
    SelectViewPoint(frontier_queue, frontier_covered, selected_viewpoint_indices, true);
  }
}

exploration_path_ns::ExplorationPath LocalCoveragePlanner::SolveTSP(const std::vector<int> &selected_viewpoint_indices, std::vector<int> &ordered_viewpoint_indices)

{
  // nav_msgs::msg::Path tsp_path;
  exploration_path_ns::ExplorationPath tsp_path;

  if (selected_viewpoint_indices.empty()) {
    return tsp_path;
  }

  // Get start and end index
  int start_ind = selected_viewpoint_indices.size() - 1;
  int end_ind = selected_viewpoint_indices.size() - 1;
  int robot_ind = 0;
  int lookahead_ind = 0;

  for (int i = 0; i < selected_viewpoint_indices.size(); i++) {
    if (selected_viewpoint_indices[i] == start_viewpoint_ind_) {
      start_ind = i;
    }
    if (selected_viewpoint_indices[i] == end_viewpoint_ind_) {
      end_ind = i;
    }
    if (selected_viewpoint_indices[i] == robot_viewpoint_ind_) {
      robot_ind = i;
    }
    if (selected_viewpoint_indices[i] == lookahead_viewpoint_ind_) {
      lookahead_ind = i;
    }
  }

  bool has_start_end_dummy = start_ind != end_ind;
  bool has_robot_lookahead_dummy = robot_ind != lookahead_ind;

  // Get distance matrix
  int node_size;
  if (has_start_end_dummy && has_robot_lookahead_dummy) {
    node_size = selected_viewpoint_indices.size() + 2;
  } else if (has_start_end_dummy || has_robot_lookahead_dummy) {
    node_size = selected_viewpoint_indices.size() + 1;
  } else {
    node_size = selected_viewpoint_indices.size();
  }
  misc_utils_ns::Timer find_path_timer("find path");
  find_path_timer.Start();
  std::vector<std::vector<int>> distance_matrix(node_size, std::vector<int>(node_size, 0));
  std::vector<int> tmp;
  for (int i = 0; i < selected_viewpoint_indices.size(); i++) {
    int from_ind = selected_viewpoint_indices[i];
    // int from_graph_idx = graph_index_map_[from_ind];
    for (int j = 0; j < i; j++) {
      int to_ind = selected_viewpoint_indices[j];
      nav_msgs::msg::Path path =
          viewpoint_manager_->GetViewPointShortestPath(from_ind, to_ind);
      double path_length = misc_utils_ns::GetPathLength(path);
      distance_matrix[i][j] = static_cast<int>(10 * path_length);
    }
  }

  for (int i = 0; i < selected_viewpoint_indices.size(); i++) {
    for (int j = i + 1; j < selected_viewpoint_indices.size(); j++) {
      distance_matrix[i][j] = distance_matrix[j][i];
    }
  }

  // Add a dummy node to connect the start and end nodes
  if (has_start_end_dummy && has_robot_lookahead_dummy) {
    int start_end_dummy_node_ind = node_size - 1;
    int robot_lookahead_dummy_node_ind = node_size - 2;
    for (int i = 0; i < selected_viewpoint_indices.size(); i++) {
      if (i == start_ind || i == end_ind) {
        distance_matrix[i][start_end_dummy_node_ind] = 0;
        distance_matrix[start_end_dummy_node_ind][i] = 0;
      } else {
        distance_matrix[i][start_end_dummy_node_ind] = 9999;
        distance_matrix[start_end_dummy_node_ind][i] = 9999;
      }
      if (i == robot_ind || i == lookahead_ind) {
        distance_matrix[i][robot_lookahead_dummy_node_ind] = 0;
        distance_matrix[robot_lookahead_dummy_node_ind][i] = 0;
      } else {
        distance_matrix[i][robot_lookahead_dummy_node_ind] = 9999;
        distance_matrix[robot_lookahead_dummy_node_ind][i] = 9999;
      }
    }

    distance_matrix[start_end_dummy_node_ind][robot_lookahead_dummy_node_ind] =
        9999;
    distance_matrix[robot_lookahead_dummy_node_ind][start_end_dummy_node_ind] =
        9999;
  } else if (has_start_end_dummy) {
    int end_node_ind = node_size - 1;
    for (int i = 0; i < selected_viewpoint_indices.size(); i++) {
      if (i == start_ind || i == end_ind) {
        distance_matrix[i][end_node_ind] = 0;
        distance_matrix[end_node_ind][i] = 0;
      } else {
        distance_matrix[i][end_node_ind] = 9999;
        distance_matrix[end_node_ind][i] = 9999;
      }
    }
  } else if (has_robot_lookahead_dummy) {
    int end_node_ind = node_size - 1;
    for (int i = 0; i < selected_viewpoint_indices.size(); i++) {
      if (i == robot_ind || i == lookahead_ind) {
        distance_matrix[i][end_node_ind] = 0;
        distance_matrix[end_node_ind][i] = 0;
      } else {
        distance_matrix[i][end_node_ind] = 9999;
        distance_matrix[end_node_ind][i] = 9999;
      }
    }
  }


  find_path_timer.Stop(false);
  find_path_runtime_ += find_path_timer.GetDuration(kRuntimeUnit);
  misc_utils_ns::Timer tsp_timer("tsp");
  tsp_timer.Start();

  tsp_solver_ns::DataModel data;
  data.distance_matrix = distance_matrix;
  data.depot = start_ind;

  tsp_solver_ns::TSPSolver tsp_solver(data);
  tsp_solver.Solve();

  std::vector<int> path_index;
  if (has_start_end_dummy) {
    tsp_solver.getSolutionNodeIndex(path_index, true);
  } else {
    tsp_solver.getSolutionNodeIndex(path_index, false);
  }

  // Get rid of the dummy node connecting the robot and lookahead point
  for (int i = 0; i < path_index.size(); i++) {
    if (path_index[i] >= selected_viewpoint_indices.size() ||
        path_index[i] < 0) {
      path_index.erase(path_index.begin() + i);
      i--;
    }
  }

  ordered_viewpoint_indices.clear();
  for (int i = 0; i < path_index.size(); i++) {
    ordered_viewpoint_indices.push_back(
        selected_viewpoint_indices[path_index[i]]);
  }

  // Add the end node index
  if (start_ind == end_ind && !path_index.empty()) {
    path_index.push_back(path_index[0]);
  }

  tsp_timer.Stop(false);
  tsp_runtime_ += tsp_timer.GetDuration(kRuntimeUnit);


  if (path_index.size() > 1) {
    int cur_ind;
    int next_ind;
    int from_graph_idx;
    int to_graph_idx;

    for (int i = 0; i < path_index.size() - 1; i++) {
      cur_ind = selected_viewpoint_indices[path_index[i]];
      next_ind = selected_viewpoint_indices[path_index[i + 1]];
      geometry_msgs::msg::Point cur_node_position = viewpoint_manager_->GetViewPointPosition(cur_ind);
      exploration_path_ns::Node cur_node(cur_node_position, exploration_path_ns::NodeType::LOCAL_VIEWPOINT);
      cur_node.local_viewpoint_ind_ = cur_ind;
      if (cur_ind == robot_viewpoint_ind_) {
        cur_node.type_ = exploration_path_ns::NodeType::ROBOT;
      } else if (cur_ind == lookahead_viewpoint_ind_) {
        int covered_point_num = viewpoint_manager_->GetViewPointCoveredPointNum(cur_ind);
        int covered_frontier_num = viewpoint_manager_->GetViewPointCoveredFrontierPointNum(cur_ind);
        if (covered_point_num > parameters_.kMinAddPointNum || covered_frontier_num > parameters_.kMinAddFrontierPointNum) {
          cur_node.type_ = exploration_path_ns::NodeType::LOCAL_VIEWPOINT;
        } else {
          cur_node.type_ = exploration_path_ns::NodeType::LOOKAHEAD_POINT;
        }
      } else if (cur_ind == start_viewpoint_ind_) {
        cur_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_START;
      } else if (cur_ind == end_viewpoint_ind_) {
        cur_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_END;
      }
      tsp_path.Append(cur_node);

      nav_msgs::msg::Path path_between_viewpoints = viewpoint_manager_->GetViewPointShortestPath(cur_ind, next_ind);

      if (path_between_viewpoints.poses.size() > 2) {
        for (int j = 1; j < path_between_viewpoints.poses.size() - 1; j++) {
          exploration_path_ns::Node node;
          node.type_ = exploration_path_ns::NodeType::LOCAL_VIA_POINT;
          node.local_viewpoint_ind_ = -1;
          node.position_.x() = path_between_viewpoints.poses[j].pose.position.x;
          node.position_.y() = path_between_viewpoints.poses[j].pose.position.y;
          node.position_.z() = path_between_viewpoints.poses[j].pose.position.z;
          tsp_path.Append(node);
        }
      }

      geometry_msgs::msg::Point next_node_position = viewpoint_manager_->GetViewPointPosition(next_ind);
      exploration_path_ns::Node next_node(next_node_position, exploration_path_ns::NodeType::LOCAL_VIEWPOINT);
      next_node.local_viewpoint_ind_ = next_ind;
      if (next_ind == robot_viewpoint_ind_) {
        next_node.type_ = exploration_path_ns::NodeType::ROBOT;
      } else if (next_ind == lookahead_viewpoint_ind_) {
        next_node.type_ = exploration_path_ns::NodeType::LOOKAHEAD_POINT;
      } else if (next_ind == start_viewpoint_ind_) {
        next_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_START;
      } else if (next_ind == end_viewpoint_ind_) {
        next_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_END;
      }
      tsp_path.Append(next_node);
    }
  }

  return tsp_path;
}

//修改  优化 局部路径     传入 可能存在的    多条全局路径  做全局信息的添加  一定几率减少死胡同类型网格
exploration_path_ns::ExplorationPath LocalCoveragePlanner::SolveLocalCoverageProblem_opt(
    const std::vector<exploration_path_ns::ExplorationPath>& near_localcoverage_subgrid_paths, int uncovered_point_num,
    int uncovered_frontier_point_num)
{
  exploration_path_ns::ExplorationPath local_path;  //本地路径

  find_path_runtime_ = 0;
  viewpoint_sampling_runtime_ = 0;
  tsp_runtime_ = 0;

  local_coverage_complete_ = false;

  misc_utils_ns::Timer find_path_timer("find path");
  find_path_timer.Start();

  std::vector<int> navigation_viewpoint_indices;  //导航视点  ID集合
  GetNavigationViewPointIndices_opt(near_localcoverage_subgrid_paths, navigation_viewpoint_indices);

  navigation_viewpoint_num_ = navigation_viewpoint_indices.size();

  find_path_timer.Stop(false);
  find_path_runtime_ += find_path_timer.GetDuration(kRuntimeUnit);

  // Sampling viewpoints    采样视点
  misc_utils_ns::Timer viewpoint_sampling_timer("viewpoint sampling");
  viewpoint_sampling_timer.Start();

  std::vector<bool> covered(uncovered_point_num, false);                    //  没覆盖点数量
  std::vector<bool> frontier_covered(uncovered_frontier_point_num, false);  //  没覆盖    边数量

  std::vector<int> pre_selected_viewpoint_array_indices;  //前  被选择array_indices的     数量
  std::vector<int> reused_viewpoint_indices;              //  再次选择的视点     ID
  for (auto& viewpoint_array_ind :
       last_selected_viewpoint_array_indices_)  //在上次 被选择的视点  array_indices   中     重选视点
  {
    if (viewpoint_manager_->ViewPointVisited(viewpoint_array_ind, true) ||
        !viewpoint_manager_->IsViewPointCandidate(viewpoint_array_ind, true))  //上次被选择的视点  viewpoint_array_ind
                                                                               //被访问过，或者   不是候选是视点
                                                                               //就不选择
    {
      continue;
    }
    int covered_point_num = viewpoint_manager_->GetViewPointCoveredPointNum(
        covered, viewpoint_array_ind, true);  //   每次更新得到的未覆盖点集     在当前视点下  可以覆盖的点数
    if (covered_point_num >= parameters_.kMinAddPointNum)  //覆盖的值大于  最小被添加值
    {
      reused_viewpoint_indices.push_back(viewpoint_manager_->GetViewPointInd(
          viewpoint_array_ind));  //这个点被重新选中    ,   可以覆盖很多未覆盖的点  有价值   被选上.
    }
    else if (use_frontier_)  //使用边界
    {
      int covered_frontier_point_num = viewpoint_manager_->GetViewPointCoveredFrontierPointNum(
          frontier_covered, viewpoint_array_ind, true);                       //边界点数量
      if (covered_frontier_point_num >= parameters_.kMinAddFrontierPointNum)  //覆盖的边界大于  最小天津爱  边界点数量
      {
        reused_viewpoint_indices.push_back(viewpoint_manager_->GetViewPointInd(
            viewpoint_array_ind));  //重新被选中  ,       可以覆盖很多未覆盖的点  有价值   被选上.
      }
    }
  }

  for (const auto& ind : reused_viewpoint_indices)  //  对于重新被选中 的点
  {
    int viewpoint_array_ind = viewpoint_manager_->GetViewPointArrayInd(ind);
    pre_selected_viewpoint_array_indices.push_back(viewpoint_array_ind);  //加入  预  被选中点 array_indices
  }
  for (const auto& ind : navigation_viewpoint_indices)  //对于导航点   加入前  被选中点 array_indices
  {
    int array_ind = viewpoint_manager_->GetViewPointArrayInd(ind);
    pre_selected_viewpoint_array_indices.push_back(array_ind);
  }

  // Update coverage      从预先被选中的视点判断    那些覆盖的点   会被覆盖么?    更新      covered   frontier_covered
  for (auto& viewpoint_array_ind : pre_selected_viewpoint_array_indices)
  {
    // Update covered points and frontiers  更新覆盖点和边界
    UpdateViewPointCoveredPoint(covered, viewpoint_array_ind, true);  //更新覆盖点
    if (use_frontier_)
    {
      UpdateViewPointCoveredFrontierPoint(frontier_covered, viewpoint_array_ind, true);  //更新边界覆盖点
    }
  }

  // Enqueue candidate viewpoints    让候选观点排队
  std::vector<std::pair<int, int>> queue;           //<覆盖  未覆盖点数量,ID>
  std::vector<std::pair<int, int>> frontier_queue;  //<覆盖  未覆盖点数量,ID>   且是其他候选点
  EnqueueViewpointCandidates(queue, frontier_queue, covered, frontier_covered,
                             pre_selected_viewpoint_array_indices);  //排序  覆盖 更多未覆盖点在前面

  viewpoint_sampling_timer.Stop(false, kRuntimeUnit);
  viewpoint_sampling_runtime_ += viewpoint_sampling_timer.GetDuration(kRuntimeUnit);


  std::vector<int> ordered_viewpoint_indices;
  if (!queue.empty() && queue[0].first > parameters_.kMinAddPointNum)  //排序不未空,且第一个点数量大于最小添加点.
  {
    double min_path_length = DBL_MAX;
    for (int itr = 0; itr < parameters_.kLocalPathOptimizationItrMax; itr++)  //局部规划优化 个数  10
    {
      std::vector<int> selected_viewpoint_indices_itr;

      misc_utils_ns::Timer select_viewpoint_timer("select viewpoints");
      select_viewpoint_timer.Start();

      SelectViewPoint_UseDistance(queue, covered, selected_viewpoint_indices_itr, false);  //核心 选点 并更新奖励
      SelectViewPointFromFrontierQueue(frontier_queue, frontier_covered, selected_viewpoint_indices_itr);

      for (const auto& ind : reused_viewpoint_indices)
      {
        selected_viewpoint_indices_itr.push_back(ind);
      }
      for (const auto& ind : navigation_viewpoint_indices)
      {
        selected_viewpoint_indices_itr.push_back(ind);
      }

      misc_utils_ns::UniquifyIntVector(selected_viewpoint_indices_itr);

      select_viewpoint_timer.Stop(false, kRuntimeUnit);
      viewpoint_sampling_runtime_ += select_viewpoint_timer.GetDuration(kRuntimeUnit);

      exploration_path_ns::ExplorationPath local_path_itr;
      local_path_itr = SolveTSP(selected_viewpoint_indices_itr, ordered_viewpoint_indices);

      double path_length = local_path_itr.GetLength();
      if (!local_path_itr.nodes_.empty() && path_length < min_path_length)

      {
        min_path_length = path_length;
        local_path = local_path_itr;
        last_selected_viewpoint_indices_ = ordered_viewpoint_indices;
      }
    }
  }
  else
  {
    misc_utils_ns::Timer select_viewpoint_timer("viewpoint sampling");
    select_viewpoint_timer.Start();

    std::vector<int> selected_viewpoint_indices_itr;

    for (const auto& ind : reused_viewpoint_indices)
    {
      selected_viewpoint_indices_itr.push_back(ind);
    }
    SelectViewPointFromFrontierQueue(frontier_queue, frontier_covered, selected_viewpoint_indices_itr);

    if (selected_viewpoint_indices_itr.empty())
    {
      local_coverage_complete_ = true;
    }

    for (const auto& ind : navigation_viewpoint_indices)
    {
      selected_viewpoint_indices_itr.push_back(ind);
    }

    misc_utils_ns::UniquifyIntVector(selected_viewpoint_indices_itr); 

    select_viewpoint_timer.Stop(false, kRuntimeUnit);
    viewpoint_sampling_runtime_ += select_viewpoint_timer.GetDuration(kRuntimeUnit);

    local_path = SolveTSP(selected_viewpoint_indices_itr, ordered_viewpoint_indices);

    last_selected_viewpoint_indices_ = ordered_viewpoint_indices;
  }

  last_selected_viewpoint_array_indices_.clear();
  for (const auto& ind : last_selected_viewpoint_indices_)
  {
    int array_ind = viewpoint_manager_->GetViewPointArrayInd(ind);
    last_selected_viewpoint_array_indices_.push_back(array_ind);
  }

  int viewpoint_num = viewpoint_manager_->GetViewPointNum();
  for (int i = 0; i < viewpoint_num; i++)
  {
    viewpoint_manager_->SetViewPointSelected(i, false, true);
  }
  for (const auto& viewpoint_index : last_selected_viewpoint_indices_)
  {
    if (viewpoint_index != robot_viewpoint_ind_ && viewpoint_index != start_viewpoint_ind_ &&
        viewpoint_index != end_viewpoint_ind_ && viewpoint_index != lookahead_viewpoint_ind_)
    {
      viewpoint_manager_->SetViewPointSelected(viewpoint_index, true);
    }
  }
  return local_path;
}


// 由全局路径    生成的  高分辨率的局部  导航路径
void LocalCoveragePlanner::GetGlobalLocalPath(const exploration_path_ns::ExplorationPath& global_path,
                                              exploration_path_ns::ExplorationPath& global_local_path)
{
  int Navigation_viewpoint_ind = GetBoundaryViewpointIndices_far(global_path);
  nav_msgs::msg::Path path_between_viewpoints = viewpoint_manager_->GetViewPointShortestPath(robot_viewpoint_ind_, Navigation_viewpoint_ind);
  global_local_path.Append(global_path.nodes_[0]);
  
  if (path_between_viewpoints.poses.size() > 2)
  {
    for (int j = 1; j < path_between_viewpoints.poses.size() - 1; j++)
    {
      exploration_path_ns::Node node;
      node.type_ = exploration_path_ns::NodeType::LOCAL_VIA_POINT;
      node.local_viewpoint_ind_ = -1;
      node.position_.x() = path_between_viewpoints.poses[j].pose.position.x;
      node.position_.y() = path_between_viewpoints.poses[j].pose.position.y;
      node.position_.z() = path_between_viewpoints.poses[j].pose.position.z;
      global_local_path.Append(node);
    }
    exploration_path_ns::Node Navigation_node;
    Navigation_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_END;
    Navigation_node.position_.x() = path_between_viewpoints.poses.back().pose.position.x;
    Navigation_node.position_.y() = path_between_viewpoints.poses.back().pose.position.y;
    Navigation_node.position_.z() = path_between_viewpoints.poses.back().pose.position.z;
    global_local_path.Append(Navigation_node);
  }
}


bool LocalCoveragePlanner::GetGlobalLocalPath_single(const exploration_path_ns::ExplorationPath& global_path,exploration_path_ns::ExplorationPath& global_local_path)
{
  int Navigation_viewpoint_ind = GetBoundaryViewpointIndices_opt(global_path); 
  nav_msgs::msg::Path path_between_viewpoints = viewpoint_manager_->GetViewPointShortestPath(robot_viewpoint_ind_, Navigation_viewpoint_ind);
  exploration_path_ns::Node robot_node;
  robot_node = global_path.nodes_[0];
  robot_node.type_ = exploration_path_ns::NodeType::ROBOT;
  global_local_path.Append(robot_node);
  if (path_between_viewpoints.poses.size() > 2)
  {
    for (int j = 1; j < path_between_viewpoints.poses.size() - 1; j++)
    {
      exploration_path_ns::Node node;
      node.type_ = exploration_path_ns::NodeType::LOCAL_VIA_POINT;
      node.local_viewpoint_ind_ = -1;
      node.position_.x() = path_between_viewpoints.poses[j].pose.position.x;
      node.position_.y() = path_between_viewpoints.poses[j].pose.position.y;
      node.position_.z() = path_between_viewpoints.poses[j].pose.position.z;
      global_local_path.Append(node);
    }
    exploration_path_ns::Node Navigation_node;
    Navigation_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_END;
    Navigation_node.position_.x() = path_between_viewpoints.poses.back().pose.position.x;
    Navigation_node.position_.y() = path_between_viewpoints.poses.back().pose.position.y;
    Navigation_node.position_.z() = path_between_viewpoints.poses.back().pose.position.z;
    global_local_path.Append(Navigation_node);

    for (int j = path_between_viewpoints.poses.size() - 2; j >= 1; j--)
    {
      exploration_path_ns::Node node;
      node.type_ = exploration_path_ns::NodeType::LOCAL_VIA_POINT;
      node.local_viewpoint_ind_ = -1;
      node.position_.x() = path_between_viewpoints.poses[j].pose.position.x;
      node.position_.y() = path_between_viewpoints.poses[j].pose.position.y;
      node.position_.z() = path_between_viewpoints.poses[j].pose.position.z;
      global_local_path.Append(node);
    }
    global_local_path.Append(robot_node);
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LocalCoveragePlanner::GetSelectedViewPointVisCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
  cloud->clear();
  for (const auto &viewpoint_index : last_selected_viewpoint_indices_) {
    geometry_msgs::msg::Point position =
        viewpoint_manager_->GetViewPointPosition(viewpoint_index);
    pcl::PointXYZI point;
    point.x = position.x;
    point.y = position.y;
    point.z = position.z;
    if (viewpoint_index == robot_viewpoint_ind_) {
      point.intensity = 0.0;
    } else if (viewpoint_index == start_viewpoint_ind_) {
      point.intensity = 1.0;
    } else if (viewpoint_index == end_viewpoint_ind_) {
      point.intensity = 2.0;
    } else {
      point.intensity = 3.0;
    }
    cloud->points.push_back(point);
  }
}

} // namespace local_coverage_planner_ns