/**
 * @file skeleton_graph.h
 * @brief Topological skeleton graph for fast global path planning.
 *
 * Builds a compact graph where each node represents a grid cell,
 * and edges represent cell-to-cell connectivity.
 * V = number of active cells (~100-200) vs KeyposeGraph V (~600-6000).
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/msg/marker.hpp>

namespace grid_world_ns {
class GridWorld;
}

namespace keypose_graph_ns {
class KeyposeGraph;
}

namespace merger_graph_ns {
class MergerGraph;
}

namespace skeleton_graph_ns {

struct SkeletonNode {
  int cell_index_;     // 对应的网格单元 ID
  Eigen::Vector3d position_;  // 单元中心位置

  SkeletonNode() : cell_index_(-1), position_(0, 0, 0) {}
  SkeletonNode(int ci, const Eigen::Vector3d& pos) : cell_index_(ci), position_(pos) {}
};

class SkeletonGraph {
 public:
  explicit SkeletonGraph();
  ~SkeletonGraph() = default;

  // ---- Build / Update ----

  /** Build the skeleton graph from GridWorld's cell connectivity. */
  void BuildFromGridWorld(
      grid_world_ns::GridWorld& grid_world,
      const merger_graph_ns::MergerGraph& merger_graph);

  /** Incremental update: add new cells / update connectivity. */
  void UpdateFromGridWorld(grid_world_ns::GridWorld& grid_world);

  /** Force rebuild from scratch next time. */
  void MarkDirty() { dirty_ = true; }

  // ---- Query ----

  /** Get the closest skeleton node index for a given position. */
  int GetClosestNodeIndex(const geometry_msgs::msg::Point& point) const;

  /** Check if a cell is reachable (represented in the graph). */
  bool IsCellInGraph(int cell_index) const;

  /** Check if a position is reachable (within dist_threshold of a node). */
  bool IsPositionReachable(const geometry_msgs::msg::Point& point,
                           double dist_threshold) const;

  /** Get shortest path distance between two cells.
   *  If use_cached is true, uses precomputed all-pairs distances. */
  double GetShortestPathDist(int from_cell, int to_cell, bool use_cached = true);

  /** Get shortest path as a nav_msgs::msg::Path between two positions.
   *  Returns false if no path exists. */
  bool GetShortestPath(const geometry_msgs::msg::Point& start,
                       const geometry_msgs::msg::Point& goal,
                       bool get_path,
                       nav_msgs::msg::Path& path,
                       const merger_graph_ns::MergerGraph& merger_graph);

  /** Get all pairs shortest path matrix. */
  const std::vector<std::vector<double>>& GetAllPairsDist() const {
    return all_pairs_dist_;
  }

  // ---- Stats ----

  int GetNodeNum() const { return nodes_.size(); }
  int GetEdgeNum() const;
  bool IsDirty() const { return dirty_; }

  /** Enable/disable skeleton graph usage. */
  static bool enabled_;

  /** Get visualization markers for RViz.
   *  Safe to call when graph is empty (returns empty markers). */
  void GetMarker(visualization_msgs::msg::Marker& node_marker,
                 visualization_msgs::msg::Marker& edge_marker) const;

 private:
  void RebuildKDTree();
  void ComputeAllPairsShortestPaths();
  void BuildFromScratch(grid_world_ns::GridWorld& grid_world);

  std::vector<SkeletonNode> nodes_;
  std::vector<std::vector<int>> graph_;         // adjacency list
  std::vector<std::vector<double>> edge_dist_;  // edge weights

  // index maps
  std::unordered_map<int, int> cell_to_node_;   // cell_index -> skeleton node index

  // KD-tree for fast nearest-neighbour
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr kdtree_cloud_;

  // Cached all-pairs shortest paths (Floyd-Warshall)
  std::vector<std::vector<double>> all_pairs_dist_;
  bool all_pairs_dirty_ = true;
  bool dirty_ = true;

  // Build tracking
  int last_cell_count_ = 0;  // used to detect when full rebuild is needed
};

}  // namespace skeleton_graph_ns
