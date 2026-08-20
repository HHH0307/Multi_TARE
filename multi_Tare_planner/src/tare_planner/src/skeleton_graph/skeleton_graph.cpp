#include "skeleton_graph/skeleton_graph.h"

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <climits>
#include <cmath>
#include <queue>
#include <set>

#include "grid_world/grid_world.h"
#include "utils/misc_utils.h"

namespace skeleton_graph_ns {

bool SkeletonGraph::enabled_ = true;

SkeletonGraph::SkeletonGraph()
    : kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZI>())
    , kdtree_cloud_(new pcl::PointCloud<pcl::PointXYZI>()) {}

// ---- Build ----

void SkeletonGraph::BuildFromGridWorld(
    grid_world_ns::GridWorld& grid_world,
    merger_graph_ns::MergerGraph& merger_graph) {
  if (!enabled_) return;
  BuildFromScratch(grid_world, merger_graph);
}

// 阶段 3.1: 从 cell 的 GetMergerGraphNodeIndices() 中选最靠近 cell 中心的 connected 节点
int SkeletonGraph::SelectRepresentativeMergerNode(
    grid_world_ns::GridWorld& grid_world,
    merger_graph_ns::MergerGraph& merger_graph,
    int cell_ind) {
  // cell 中心位置
  geometry_msgs::msg::Point cell_pos = grid_world.GetCellPosition(cell_ind);
  Eigen::Vector3d cell_center(cell_pos.x, cell_pos.y, cell_pos.z);

  // 获取 cell 内的所有 merger_graph 节点
  std::vector<int> mg_node_inds = grid_world.GetCellMergerGraphNodeIndices(cell_ind);
  int best_mg_ind = -1;
  double best_dist = DBL_MAX;
  for (int mg_ind : mg_node_inds) {
    if (mg_ind < 0 || mg_ind >= merger_graph.GetNodeNum()) continue;
    // 只选 connected 节点（保证可通行）
    if (!merger_graph.GetNodeIsConnected(mg_ind)) continue;
    geometry_msgs::msg::Point mg_pos = merger_graph.GetNodePosition(mg_ind);
    Eigen::Vector3d p(mg_pos.x, mg_pos.y, mg_pos.z);
    double d = (p - cell_center).norm();
    if (d < best_dist) {
      best_dist = d;
      best_mg_ind = mg_ind;
    }
  }
  return best_mg_ind;
}

void SkeletonGraph::BuildFromScratch(
    grid_world_ns::GridWorld& grid_world,
    merger_graph_ns::MergerGraph& merger_graph) {
  // 阶段 2.4: 性能计时
  auto t_start = std::chrono::high_resolution_clock::now();

  nodes_.clear();
  graph_.clear();
  graph_set_.clear();  // 阶段 2.2
  edge_dist_.clear();
  cell_to_node_.clear();
  all_pairs_dist_.clear();
  dijkstra_cache_.clear();
  all_pairs_dirty_ = true;
  dirty_ = false;
  degraded_ = false;

  // 阶段 3.1: 收集 EXPLORING+COVERED 的 cell（world 视角）
  std::vector<int> exploring_cells;
  grid_world.GetExploringAndCoveredCellIndicesWorld(exploring_cells);

  // Phase 1: 为每个 cell 选取 merger_graph 代表节点并创建骨架节点
  for (int cell_ind : exploring_cells) {
    AddCellNode(grid_world, merger_graph, cell_ind);
  }

  // Phase 2: 构建邻接边（基于 GridWorld 的 cell 邻接关系）
  for (int cell_ind : exploring_cells) {
    AddEdgesForCell(grid_world, cell_ind);
  }

  // 阶段 3.2: 度数过滤骨架化——移除非边界叶子节点
  PruneLeafNodes(grid_world);

  // 阶段 2.5: 大节点数降级保护
  if (static_cast<int>(nodes_.size()) > kMaxSkeletonNodeNum_) {
    degraded_ = true;
  }

  // Phase 3: Rebuild KD-tree and compute all-pairs shortest paths
  if (!nodes_.empty()) {
    RebuildKDTree();
    if (static_cast<int>(nodes_.size()) <= kMaxFloydWarshallNodeNum_ && !degraded_) {
      ComputeAllPairsShortestPaths();
    } else {
      all_pairs_dist_.clear();
      all_pairs_dirty_ = true;
    }
  }
  last_cell_count_ = exploring_cells.size();
  last_cell_set_.clear();
  last_cell_set_.insert(exploring_cells.begin(), exploring_cells.end());

  // 阶段 2.4: 统计
  auto t_end = std::chrono::high_resolution_clock::now();
  double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  stats_.last_build_ms = ms;
  stats_.total_build_ms += ms;
  stats_.build_count++;
  stats_.node_num = GetNodeNum();
  stats_.edge_num = GetEdgeNum();
}

// 阶段 3.1: 为 cell 创建骨架节点（位置 = merger_graph 代表节点的实际位置）
void SkeletonGraph::AddCellNode(grid_world_ns::GridWorld& grid_world,
                                 merger_graph_ns::MergerGraph& merger_graph,
                                 int cell_ind) {
  if (cell_to_node_.find(cell_ind) != cell_to_node_.end()) return;  // 已存在
  auto status = grid_world.GetCellStatus_world(cell_ind);
  if (status != grid_world_ns::CellStatus::EXPLORING &&
      status != grid_world_ns::CellStatus::COVERED) {
    return;
  }
  // 阶段 3.1: 选取 merger_graph 代表节点
  int mg_node_ind = SelectRepresentativeMergerNode(grid_world, merger_graph, cell_ind);

  Eigen::Vector3d node_pos;
  if (mg_node_ind >= 0) {
    // 用 merger_graph 节点的实际位置
    geometry_msgs::msg::Point p = merger_graph.GetNodePosition(mg_node_ind);
    node_pos = Eigen::Vector3d(p.x, p.y, p.z);
  } else {
    // 回退: 该 cell 内无 connected merger_graph 节点，用 cell 中心
    //   这种 cell 会被 PruneLeafNodes 或查询失败时自然过滤掉
    geometry_msgs::msg::Point p = grid_world.GetCellPosition(cell_ind);
    node_pos = Eigen::Vector3d(p.x, p.y, p.z);
  }

  SkeletonNode node(cell_ind, node_pos, mg_node_ind);
  int node_idx = nodes_.size();
  nodes_.push_back(node);
  graph_.push_back({});
  graph_set_.push_back({});
  edge_dist_.push_back({});
  cell_to_node_[cell_ind] = node_idx;
}

void SkeletonGraph::AddEdgesForCell(grid_world_ns::GridWorld& grid_world, int cell_ind) {
  auto it = cell_to_node_.find(cell_ind);
  if (it == cell_to_node_.end()) return;
  int from_node = it->second;

  std::vector<int> connected = grid_world.GetCellConnectedCellIndices(cell_ind);
  for (int neighbor_cell : connected) {
    auto nit = cell_to_node_.find(neighbor_cell);
    if (nit == cell_to_node_.end()) continue;
    int to_node = nit->second;

    // Skip self-loops
    if (from_node == to_node) continue;
    // 阶段 2.2: O(1) 去重检查
    if (graph_set_[from_node].count(to_node)) continue;

    // Add edge (undirected)
    //   阶段 3.1: 边权用两代表节点的实际位置（merger_graph 节点位置）的直线距离。
    //   Floyd-Warshall/Dijkstra 会自动累加边权得到正确的最短路径距离，
    //   因此这里用直线距离近似即可（节点位置已是 merger_graph 实际位置，比 cell 中心准确）。
    double dist = (nodes_[from_node].position_ - nodes_[to_node].position_).norm();
    graph_[from_node].push_back(to_node);
    graph_[to_node].push_back(from_node);
    edge_dist_[from_node].push_back(dist);
    edge_dist_[to_node].push_back(dist);
    graph_set_[from_node].insert(to_node);
    graph_set_[to_node].insert(from_node);
  }
}

// 阶段 3.2: 度数过滤骨架化
//   移除度数 = 1 且其 cell 不是边界 cell（没有 UNSEEN 邻居）的叶子节点
//   注意: 删除节点需要索引重排，这里采用"标记 + 重建"的简化方式
void SkeletonGraph::PruneLeafNodes(grid_world_ns::GridWorld& grid_world) {
  if (nodes_.empty()) return;

  // 判断 cell 是否是边界 cell（有 UNSEEN 邻居，可能是探索前沿）
  auto is_frontier_cell = [&](int cell_ind) -> bool {
    std::vector<int> connected = grid_world.GetCellConnectedCellIndices(cell_ind);
    // 同时检查 cell 邻接的 6-邻域是否有 UNSEEN
    //   注意: GetCellConnectedCellIndices 返回的是已连接的 cell（可能不含 UNSEEN）
    //   这里简化: 若 cell 的 connected 列表 < 6（6-邻域未满），视为边界
    //   TODO(阶段3.2): 可改为检查实际邻居 cell 状态
    return connected.size() < 6;
  };

  // 找出所有要删除的叶子节点
  std::vector<int> to_remove;
  for (size_t i = 0; i < nodes_.size(); i++) {
    if (graph_[i].size() == 1) {  // 度数 = 1
      int cell_ind = nodes_[i].cell_index_;
      if (!is_frontier_cell(cell_ind)) {
        to_remove.push_back(static_cast<int>(i));
      }
    }
  }

  if (to_remove.empty()) return;

  // 简化实现: 重建 nodes/graph/edge_dist/cell_to_node，跳过 to_remove 中的节点
  //   注意: 这种实现会改变节点索引，需要更新 cell_to_node_ 和 graph_set_
  std::unordered_set<int> remove_set(to_remove.begin(), to_remove.end());

  std::vector<SkeletonNode> new_nodes;
  std::vector<std::vector<int>> new_graph;
  std::vector<std::vector<double>> new_edge_dist;
  std::unordered_map<int, int> new_cell_to_node;
  std::vector<std::unordered_set<int>> new_graph_set;

  // 旧索引 -> 新索引的映射
  std::vector<int> old_to_new(nodes_.size(), -1);
  int new_idx = 0;
  for (size_t i = 0; i < nodes_.size(); i++) {
    if (remove_set.count(static_cast<int>(i))) continue;
    old_to_new[i] = new_idx;
    new_nodes.push_back(nodes_[i]);
    new_graph.push_back({});
    new_edge_dist.push_back({});
    new_graph_set.push_back({});
    new_cell_to_node[nodes_[i].cell_index_] = new_idx;
    new_idx++;
  }

  // 重建边（跳过涉及被删除节点的边）
  for (size_t i = 0; i < nodes_.size(); i++) {
    if (old_to_new[i] < 0) continue;
    int new_i = old_to_new[i];
    for (size_t j = 0; j < graph_[i].size(); j++) {
      int old_neighbor = graph_[i][j];
      if (old_to_new[old_neighbor] < 0) continue;
      int new_neighbor = old_to_new[old_neighbor];
      // 去重（双向边只添加一次）
      if (new_graph_set[new_i].count(new_neighbor)) continue;
      new_graph[new_i].push_back(new_neighbor);
      new_edge_dist[new_i].push_back(edge_dist_[i][j]);
      new_graph_set[new_i].insert(new_neighbor);
    }
  }

  nodes_ = std::move(new_nodes);
  graph_ = std::move(new_graph);
  edge_dist_ = std::move(new_edge_dist);
  graph_set_ = std::move(new_graph_set);
  cell_to_node_ = std::move(new_cell_to_node);
  // all_pairs_dist_ 会在 BuildFromScratch 后续步骤中重算
}

// 阶段 2.1 + 3.3b: 增量更新
bool SkeletonGraph::IncrementalUpdate(
    grid_world_ns::GridWorld& grid_world,
    merger_graph_ns::MergerGraph& merger_graph,
    const std::vector<int>& added_cells,
    const std::vector<int>& removed_cells) {
  auto t_start = std::chrono::high_resolution_clock::now();

  bool has_removed = !removed_cells.empty();
  bool has_added = !added_cells.empty();

  if (!has_added && !has_removed) {
    return false;  // 无变化
  }

  if (has_removed) {
    // 删除节点需要索引重排，降级为全量重建
    //   注意: BuildFromScratch 内部已记录 build 统计，此处不重复记录
    BuildFromScratch(grid_world, merger_graph);
    return true;
  }

  // 只有 added: 增量添加节点
  for (int cell_ind : added_cells) {
    AddCellNode(grid_world, merger_graph, cell_ind);
  }
  for (int cell_ind : added_cells) {
    AddEdgesForCell(grid_world, cell_ind);
  }

  // 阶段 3.2: 增量添加后不做度数过滤（成本高），留给下次全量重建
  //   注意: 这可能导致叶子节点累积，但影响有限（边权查询仍正确）

  // 阶段 2.5: 增量添加后检查是否超过节点上限
  if (static_cast<int>(nodes_.size()) > kMaxSkeletonNodeNum_) {
    degraded_ = true;
  }

  if (!nodes_.empty()) {
    RebuildKDTree();
    if (static_cast<int>(nodes_.size()) <= kMaxFloydWarshallNodeNum_ && !degraded_) {
      ComputeAllPairsShortestPaths();
    } else {
      all_pairs_dist_.clear();
      all_pairs_dirty_ = true;
      dijkstra_cache_.clear();
    }
  }

  last_cell_count_ = nodes_.size();
  last_cell_set_.clear();
  std::vector<int> cur_cells;
  grid_world.GetExploringAndCoveredCellIndicesWorld(cur_cells);
  last_cell_set_.insert(cur_cells.begin(), cur_cells.end());

  auto t_end = std::chrono::high_resolution_clock::now();
  double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  stats_.last_update_ms = ms;
  stats_.total_update_ms += ms;
  stats_.incremental_update_count++;
  stats_.node_num = GetNodeNum();
  stats_.edge_num = GetEdgeNum();

  return true;
}

void SkeletonGraph::UpdateFromGridWorld(
    grid_world_ns::GridWorld& grid_world,
    merger_graph_ns::MergerGraph& merger_graph) {
  if (!enabled_) return;

  // 修复 1.4 + 阶段 2.1: 计算 added/removed 集合
  std::vector<int> cur_cells;
  grid_world.GetExploringAndCoveredCellIndicesWorld(cur_cells);
  std::unordered_set<int> cur_set(cur_cells.begin(), cur_cells.end());

  std::vector<int> added_cells, removed_cells;
  for (int c : cur_cells) {
    if (last_cell_set_.find(c) == last_cell_set_.end()) {
      added_cells.push_back(c);
    }
  }
  for (int c : last_cell_set_) {
    if (cur_set.find(c) == cur_set.end()) {
      removed_cells.push_back(c);
    }
  }

  bool changed = !added_cells.empty() || !removed_cells.empty();

  if (dirty_) {
    BuildFromScratch(grid_world, merger_graph);
  } else if (changed) {
    IncrementalUpdate(grid_world, merger_graph, added_cells, removed_cells);
  }
}

void SkeletonGraph::RebuildKDTree() {
  kdtree_cloud_->clear();
  for (size_t i = 0; i < nodes_.size(); i++) {
    pcl::PointXYZI pt;
    pt.x = nodes_[i].position_.x();
    pt.y = nodes_[i].position_.y();
    pt.z = nodes_[i].position_.z();
    pt.intensity = i;
    kdtree_cloud_->points.push_back(pt);
  }
  if (!kdtree_cloud_->points.empty()) {
    kdtree_->setInputCloud(kdtree_cloud_);
  }
}

// ---- Query ----

int SkeletonGraph::GetClosestNodeIndex(
    const geometry_msgs::msg::Point& point) const {
  if (kdtree_cloud_->points.empty()) return -1;

  pcl::PointXYZI search_pt;
  search_pt.x = point.x;
  search_pt.y = point.y;
  search_pt.z = point.z;

  std::vector<int> indices(1);
  std::vector<float> sq_dists(1);
  int found = kdtree_->nearestKSearch(search_pt, 1, indices, sq_dists);
  if (found > 0 && !indices.empty() && indices[0] >= 0 &&
      indices[0] < static_cast<int>(kdtree_cloud_->points.size())) {
    return static_cast<int>(kdtree_cloud_->points[indices[0]].intensity);
  }
  return -1;
}

bool SkeletonGraph::IsCellInGraph(int cell_index) const {
  return cell_to_node_.find(cell_index) != cell_to_node_.end();
}

bool SkeletonGraph::IsPositionReachable(
    const geometry_msgs::msg::Point& point,
    double dist_threshold) const {
  int idx = GetClosestNodeIndex(point);
  if (idx < 0 || idx >= static_cast<int>(nodes_.size())) return false;
  Eigen::Vector3d pos(point.x, point.y, point.z);
  double dist = (nodes_[idx].position_ - pos).norm();
  return dist < dist_threshold;
}

std::vector<double> SkeletonGraph::DijkstraFromNode(int source_node) {
  // 阶段 2.3: LRU 缓存查询
  auto cache_it = dijkstra_cache_.find(source_node);
  if (cache_it != dijkstra_cache_.end()) {
    cache_it->second.last_use_tick = ++dijkstra_tick_;
    return cache_it->second.dist;
  }

  // 缓存未命中，跑 Dijkstra
  int V = static_cast<int>(nodes_.size());
  std::vector<double> dist(V, DBL_MAX);
  if (source_node < 0 || source_node >= V) return dist;

  typedef std::pair<double, int> iPair;
  std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq;
  dist[source_node] = 0;
  pq.push({0.0, source_node});

  while (!pq.empty()) {
    auto top = pq.top();
    pq.pop();
    double d = top.first;
    int u = top.second;
    if (d > dist[u]) continue;  // stale entry

    for (size_t i = 0; i < graph_[u].size(); i++) {
      int v = graph_[u][i];
      double w = edge_dist_[u][i];
      if (dist[v] > dist[u] + w) {
        dist[v] = dist[u] + w;
        pq.push({dist[v], v});
      }
    }
  }

  // LRU 淘汰: 缓存满时淘汰 last_use_tick 最小的条目
  if (static_cast<int>(dijkstra_cache_.size()) >= kDijkstraCacheSize_ &&
      kDijkstraCacheSize_ > 0) {
    int oldest_key = -1;
    int64_t oldest_tick = INT64_MAX;
    for (const auto& kv : dijkstra_cache_) {
      if (kv.second.last_use_tick < oldest_tick) {
        oldest_tick = kv.second.last_use_tick;
        oldest_key = kv.first;
      }
    }
    if (oldest_key >= 0) {
      dijkstra_cache_.erase(oldest_key);
    }
  }

  DijkstraCacheEntry entry;
  entry.dist = dist;
  entry.last_use_tick = ++dijkstra_tick_;
  dijkstra_cache_[source_node] = entry;
  stats_.dijkstra_query_count++;
  return dist;
}

double SkeletonGraph::GetShortestPathDist(
    int from_cell, int to_cell, bool use_cached) {
  (void)use_cached;  // 当前实现: 小图查缓存矩阵, 大图用 Dijkstra+LRU; use_cached 参数保留兼容
  auto t_start = std::chrono::high_resolution_clock::now();

  stats_.dist_query_count++;

  // Guard: 降级模式或空图，直接返回 -1 让 caller 回退到 merger_graph
  if (nodes_.empty() || degraded_) {
    auto t_end = std::chrono::high_resolution_clock::now();
    stats_.total_query_ms += std::chrono::duration<double, std::milli>(t_end - t_start).count();
    return -1.0;
  }

  auto it_from = cell_to_node_.find(from_cell);
  auto it_to = cell_to_node_.find(to_cell);
  if (it_from == cell_to_node_.end() || it_to == cell_to_node_.end()) {
    auto t_end = std::chrono::high_resolution_clock::now();
    stats_.total_query_ms += std::chrono::duration<double, std::milli>(t_end - t_start).count();
    return -1.0;
  }

  int fi = it_from->second;
  int ti = it_to->second;

  // 阶段 2.3: 小图查 Floyd-Warshall 缓存矩阵；大图用 Dijkstra + LRU
  if (all_pairs_dirty_ || (!all_pairs_dist_.empty() && all_pairs_dist_.size() != nodes_.size())) {
    // 节点数变化后缓存矩阵可能失效，重新计算
    if (static_cast<int>(nodes_.size()) <= kMaxFloydWarshallNodeNum_ && !degraded_) {
      ComputeAllPairsShortestPaths();
    }
  }

  double result = -1.0;
  if (!all_pairs_dist_.empty()) {
    // 小图模式: O(1) 查询
    if (fi < 0 || fi >= static_cast<int>(all_pairs_dist_.size()) ||
        ti < 0 || ti >= static_cast<int>(all_pairs_dist_.size())) {
      auto t_end = std::chrono::high_resolution_clock::now();
      stats_.total_query_ms += std::chrono::duration<double, std::milli>(t_end - t_start).count();
      return -1.0;
    }
    result = all_pairs_dist_[fi][ti];
    if (result >= DBL_MAX * 0.5) result = -1.0;
  } else {
    // 大图模式: Dijkstra 从 fi 出发
    std::vector<double> dist = DijkstraFromNode(fi);
    if (ti >= 0 && ti < static_cast<int>(dist.size())) {
      result = dist[ti];
      if (result >= DBL_MAX * 0.5) result = -1.0;
    }
  }

  if (result > 0) {
    stats_.dist_query_hit_count++;
  }

  auto t_end = std::chrono::high_resolution_clock::now();
  stats_.total_query_ms += std::chrono::duration<double, std::milli>(t_end - t_start).count();
  return result;
}

bool SkeletonGraph::GetShortestPath(
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& goal,
    bool get_path,
    nav_msgs::msg::Path& path,
    merger_graph_ns::MergerGraph& merger_graph) {
  // 阶段 3.3a: 若骨架图为空或降级，直接回退到 merger_graph 全程 A*
  if (nodes_.empty() || degraded_) {
    if (get_path) {
      return merger_graph.GetShortestPath(start, goal, true, path, true) >= 0;
    }
    return false;
  }

  int start_node = GetClosestNodeIndex(start);
  int goal_node = GetClosestNodeIndex(goal);
  if (start_node < 0 || goal_node < 0) {
    // 骨架图找不到对应节点，回退到 merger_graph
    if (get_path) {
      return merger_graph.GetShortestPath(start, goal, true, path, true) >= 0;
    }
    return false;
  }

  // A* on skeleton graph
  double INF = 1e9;
  typedef std::pair<double, int> iPair;
  std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq;
  std::vector<double> g(nodes_.size(), INF);
  std::vector<double> f(nodes_.size(), INF);
  std::vector<int> prev(nodes_.size(), -1);
  std::vector<bool> in_pq(nodes_.size(), false);

  g[start_node] = 0;
  f[start_node] = (nodes_[start_node].position_ - nodes_[goal_node].position_).norm();
  pq.push({f[start_node], start_node});
  in_pq[start_node] = true;

  bool found = false;
  while (!pq.empty()) {
    int u = pq.top().second;
    pq.pop();
    in_pq[u] = false;
    if (u == goal_node) { found = true; break; }

    for (size_t i = 0; i < graph_[u].size(); i++) {
      int v = graph_[u][i];
      double d = edge_dist_[u][i];
      if (g[v] > g[u] + d) {
        prev[v] = u;
        g[v] = g[u] + d;
        f[v] = g[v] + (nodes_[v].position_ - nodes_[goal_node].position_).norm();
        if (!in_pq[v]) {
          pq.push({f[v], v});
          in_pq[v] = true;
        }
      }
    }
  }

  if (!found) {
    // 骨架图找不到路径，回退到 merger_graph
    if (get_path) {
      return merger_graph.GetShortestPath(start, goal, true, path, true) >= 0;
    }
    return false;
  }

  if (get_path) {
    // 阶段 3.3a: 得到骨架图 cell 序列，再用 merger_graph 细化每对相邻 cell 的实际路径
    std::vector<int> node_seq;
    for (int u = goal_node; u != -1; u = prev[u]) {
      node_seq.push_back(u);
    }
    std::reverse(node_seq.begin(), node_seq.end());

    path.poses.clear();

    // 起点: 从 start 到第一个骨架节点的 merger_graph 路径
    if (!node_seq.empty() && nodes_[node_seq.front()].merger_node_ind_ >= 0) {
      nav_msgs::msg::Path first_leg;
      if (merger_graph.GetShortestPath(start,
          merger_graph.GetNodePosition(nodes_[node_seq.front()].merger_node_ind_),
          true, first_leg, true) >= 0) {
        // 添加 first_leg 除最后一个点外的所有点（避免与下一段重复）
        for (size_t i = 0; i + 1 < first_leg.poses.size(); i++) {
          path.poses.push_back(first_leg.poses[i]);
        }
      }
    } else {
      // 第一个节点无对应 merger_graph 节点，直接添加 start
      geometry_msgs::msg::PoseStamped start_pose;
      start_pose.pose.position = start;
      path.poses.push_back(start_pose);
    }

    // 中间段: 每对相邻骨架节点之间的 merger_graph 路径
    for (size_t i = 0; i + 1 < node_seq.size(); i++) {
      int from_mg = nodes_[node_seq[i]].merger_node_ind_;
      int to_mg = nodes_[node_seq[i + 1]].merger_node_ind_;
      if (from_mg < 0 || to_mg < 0) {
        // 无对应 merger_graph 节点，用骨架图节点位置做直线连接
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = nodes_[node_seq[i]].position_.x();
        pose.pose.position.y = nodes_[node_seq[i]].position_.y();
        pose.pose.position.z = nodes_[node_seq[i]].position_.z();
        path.poses.push_back(pose);
        continue;
      }
      nav_msgs::msg::Path leg;
      if (merger_graph.GetShortestPath(
          merger_graph.GetNodePosition(from_mg),
          merger_graph.GetNodePosition(to_mg),
          true, leg, true) >= 0) {
        // 添加 leg 除最后一个点外的所有点（避免与下一段重复）
        for (size_t j = 0; j + 1 < leg.poses.size(); j++) {
          path.poses.push_back(leg.poses[j]);
        }
      }
    }

    // 终点: 从最后一个骨架节点到 goal 的 merger_graph 路径
    if (!node_seq.empty() && nodes_[node_seq.back()].merger_node_ind_ >= 0) {
      nav_msgs::msg::Path last_leg;
      if (merger_graph.GetShortestPath(
          merger_graph.GetNodePosition(nodes_[node_seq.back()].merger_node_ind_),
          goal, true, last_leg, true) >= 0) {
        // 添加 last_leg 除第一个点外的所有点（避免与上一段重复）
        for (size_t i = 1; i < last_leg.poses.size(); i++) {
          path.poses.push_back(last_leg.poses[i]);
        }
      } else {
        // 最后一段失败，直接添加 goal
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.pose.position = goal;
        path.poses.push_back(goal_pose);
      }
    } else {
      geometry_msgs::msg::PoseStamped goal_pose;
      goal_pose.pose.position = goal;
      path.poses.push_back(goal_pose);
    }
  }
  return found;
}

void SkeletonGraph::ComputeAllPairsShortestPaths() {
  int V = static_cast<int>(nodes_.size());
  if (V == 0) return;

  // 阶段 2.3: 大图跳过 Floyd-Warshall
  if (V > kMaxFloydWarshallNodeNum_) {
    all_pairs_dist_.clear();
    all_pairs_dirty_ = true;
    return;
  }

  all_pairs_dist_.assign(V, std::vector<double>(V, DBL_MAX));
  for (int i = 0; i < V; i++) {
    all_pairs_dist_[i][i] = 0;
    for (size_t j = 0; j < graph_[i].size(); j++) {
      int v = graph_[i][j];
      if (v >= 0 && v < V) {
        all_pairs_dist_[i][v] = std::min(all_pairs_dist_[i][v], edge_dist_[i][j]);
      }
    }
  }

  // Floyd-Warshall: O(V^3) but V is ~100-200 → 1M-8M operations
  for (int k = 0; k < V; k++) {
    for (int i = 0; i < V; i++) {
      if (all_pairs_dist_[i][k] >= DBL_MAX * 0.5) continue;
      for (int j = 0; j < V; j++) {
        if (all_pairs_dist_[k][j] >= DBL_MAX * 0.5) continue;
        double through_k = all_pairs_dist_[i][k] + all_pairs_dist_[k][j];
        if (through_k < all_pairs_dist_[i][j]) {
          all_pairs_dist_[i][j] = through_k;
        }
      }
    }
  }
  all_pairs_dirty_ = false;
}

int SkeletonGraph::GetEdgeNum() const {
  int count = 0;
  for (const auto& edges : graph_) count += edges.size();
  return count / 2;
}

void SkeletonGraph::GetMarker(visualization_msgs::msg::Marker& node_marker,
                               visualization_msgs::msg::Marker& edge_marker) const {
  node_marker.points.clear();
  edge_marker.points.clear();
  if (nodes_.empty()) return;

  for (const auto& node : nodes_) {
    geometry_msgs::msg::Point p;
    p.x = node.position_.x();
    p.y = node.position_.y();
    p.z = node.position_.z();
    node_marker.points.push_back(p);
  }

  // Deduplicate edges for LINE_LIST
  std::set<std::pair<int, int>> added;
  for (size_t i = 0; i < graph_.size(); i++) {
    for (int neighbor : graph_[i]) {
      int a = static_cast<int>(i);
      int b = neighbor;
      if (a > b) std::swap(a, b);
      if (added.count({a, b})) continue;
      added.insert({a, b});

      geometry_msgs::msg::Point p1, p2;
      p1.x = nodes_[a].position_.x(); p1.y = nodes_[a].position_.y(); p1.z = nodes_[a].position_.z();
      p2.x = nodes_[b].position_.x(); p2.y = nodes_[b].position_.y(); p2.z = nodes_[b].position_.z();
      edge_marker.points.push_back(p1);
      edge_marker.points.push_back(p2);
    }
  }
}

// 阶段 2.4: 统计重置
void SkeletonGraph::ResetStats() {
  stats_ = Stats{};
}

}  // namespace skeleton_graph_ns
