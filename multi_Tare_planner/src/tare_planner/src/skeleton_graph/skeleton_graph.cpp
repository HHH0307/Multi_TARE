#include "skeleton_graph/skeleton_graph.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <queue>
#include <set>
#include <stack>

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
    const merger_graph_ns::MergerGraph& merger_graph) {
  if (!enabled_) return;
  BuildFromScratch(grid_world);
}

void SkeletonGraph::BuildFromScratch(
    grid_world_ns::GridWorld& grid_world) {
  nodes_.clear();
  graph_.clear();
  edge_dist_.clear();
  cell_to_node_.clear();
  all_pairs_dist_.clear();
  all_pairs_dirty_ = true;
  dirty_ = false;

  // Phase 1: Collect exploring cells and create skeleton nodes
  std::unordered_set<int> added_cells;

  // Helper: add a node for a cell if not already added
  auto add_cell_node = [&](int cell_ind) {
    if (added_cells.count(cell_ind)) return;
    auto status = grid_world.GetCellStatus_world(cell_ind);
    if (status == grid_world_ns::CellStatus::EXPLORING ||
        status == grid_world_ns::CellStatus::COVERED) {
      geometry_msgs::msg::Point pos = grid_world.GetCellPosition(cell_ind);
      SkeletonNode node(cell_ind, Eigen::Vector3d(pos.x, pos.y, pos.z));
      int node_idx = nodes_.size();
      nodes_.push_back(node);
      graph_.push_back({});
      edge_dist_.push_back({});
      cell_to_node_[cell_ind] = node_idx;
      added_cells.insert(cell_ind);
    }
  };

  // Phase 2: Get exploring cells
  std::vector<int> exploring_cells;
  grid_world.GetExploringCellIndices(exploring_cells);

  for (int cell_ind : exploring_cells) {
    add_cell_node(cell_ind);
  }

  // Phase 3: Add edges using GridWorld's existing cell connectivity
  for (int cell_ind : exploring_cells) {
    auto it = cell_to_node_.find(cell_ind);
    if (it == cell_to_node_.end()) continue;
    int from_node = it->second;

    std::vector<int> connected = grid_world.GetCellConnectedCellIndices(cell_ind);
    for (int neighbor_cell : connected) {
      auto nit = cell_to_node_.find(neighbor_cell);
      if (nit == cell_to_node_.end()) continue;
      int to_node = nit->second;

      // Skip self-loops and duplicate edges
      if (from_node == to_node) continue;
      bool exists = false;
      for (int e : graph_[from_node]) {
        if (e == to_node) { exists = true; break; }
      }
      if (exists) continue;

      // Add edge (undirected)
      double dist = (nodes_[from_node].position_ - nodes_[to_node].position_).norm();
      graph_[from_node].push_back(to_node);
      graph_[to_node].push_back(from_node);
      edge_dist_[from_node].push_back(dist);
      edge_dist_[to_node].push_back(dist);
    }
  }

  // Phase 4: Rebuild KD-tree and compute all-pairs shortest paths
  if (!nodes_.empty()) {
    RebuildKDTree();
    ComputeAllPairsShortestPaths();
  }
  last_cell_count_ = exploring_cells.size();
}

void SkeletonGraph::UpdateFromGridWorld(
    grid_world_ns::GridWorld& grid_world) {
  if (!enabled_) return;

  std::vector<int> exploring_cells;
  grid_world.GetExploringCellIndices(exploring_cells);
  printf("[DEBUG] skel: %zu cells (was %zu)\n", exploring_cells.size(), last_cell_count_);
  if (exploring_cells.size() != last_cell_count_) {
    BuildFromScratch(grid_world);
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

double SkeletonGraph::GetShortestPathDist(
    int from_cell, int to_cell, bool use_cached) {
  // Guard: if skeleton is empty, fall through and let caller handle
  if (nodes_.empty()) return -1.0;  // signal "not available"

  // Ensure all-pairs is computed
  if (all_pairs_dirty_ || all_pairs_dist_.empty()) {
    ComputeAllPairsShortestPaths();
  }

  // Guard: if still empty after compute, return -1
  if (all_pairs_dist_.empty()) return -1.0;

  auto it_from = cell_to_node_.find(from_cell);
  auto it_to = cell_to_node_.find(to_cell);
  if (it_from == cell_to_node_.end() || it_to == cell_to_node_.end())
    return -1.0;

  int fi = it_from->second;
  int ti = it_to->second;
  if (fi < 0 || fi >= static_cast<int>(all_pairs_dist_.size()) ||
      ti < 0 || ti >= static_cast<int>(all_pairs_dist_.size()))
    return -1.0;

  double result = all_pairs_dist_[fi][ti];
  // If unreachable, return -1 to signal fallback to merger_graph
  if (result >= DBL_MAX * 0.5) return -1.0;
  return result;
}

bool SkeletonGraph::GetShortestPath(
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& goal,
    bool get_path,
    nav_msgs::msg::Path& path,
    const merger_graph_ns::MergerGraph& merger_graph) {
  if (nodes_.empty()) return false;

  int start_node = GetClosestNodeIndex(start);
  int goal_node = GetClosestNodeIndex(goal);
  if (start_node < 0 || goal_node < 0) return false;

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

  if (found && get_path) {
    std::vector<int> cell_seq;
    for (int u = goal_node; u != -1; u = prev[u]) {
      cell_seq.push_back(nodes_[u].cell_index_);
    }
    std::reverse(cell_seq.begin(), cell_seq.end());

    path.poses.clear();
    for (size_t i = 0; i < cell_seq.size(); i++) {
      auto it = cell_to_node_.find(cell_seq[i]);
      if (it == cell_to_node_.end()) continue;
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = nodes_[it->second].position_.x();
      pose.pose.position.y = nodes_[it->second].position_.y();
      pose.pose.position.z = nodes_[it->second].position_.z();
      path.poses.push_back(pose);
    }
  }
  return found;
}

void SkeletonGraph::ComputeAllPairsShortestPaths() {
  int V = static_cast<int>(nodes_.size());
  if (V == 0) return;

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

}  // namespace skeleton_graph_ns
