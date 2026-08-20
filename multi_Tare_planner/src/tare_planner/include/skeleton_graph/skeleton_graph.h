/**
 * @file skeleton_graph.h
 * @brief Topological skeleton graph for fast global path planning.
 *
 * Builds a compact graph where each node represents a grid cell,
 * and edges represent cell-to-cell connectivity.
 * V = number of active cells (~100-200) vs KeyposeGraph V (~600-6000).
 *
 * 阶段 2 改进:
 *   - 增量更新: 只增删变化的 cell 节点/边，不全量重建
 *   - 邻接集 std::unordered_set 做 O(1) 去重
 *   - 大图 (> kMaxFloydWarshallNodeNum) 改用按需 Dijkstra + LRU 缓存
 *   - 性能计时与命中率统计
 *   - 大节点数降级保护 (> kMaxSkeletonNodeNum 禁用骨架图)
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cstdint>

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
  Eigen::Vector3d position_;  // 节点位置（阶段3: = merger_graph 代表节点的实际位置，非 cell 中心）
  int merger_node_ind_;  // 阶段3.1: 该骨架节点代表的 merger_graph 节点索引（-1 表示无对应）

  SkeletonNode() : cell_index_(-1), position_(0, 0, 0), merger_node_ind_(-1) {}
  SkeletonNode(int ci, const Eigen::Vector3d& pos)
      : cell_index_(ci), position_(pos), merger_node_ind_(-1) {}
  SkeletonNode(int ci, const Eigen::Vector3d& pos, int mg_ind)
      : cell_index_(ci), position_(pos), merger_node_ind_(mg_ind) {}
};

class SkeletonGraph {
 public:
  explicit SkeletonGraph();
  ~SkeletonGraph() = default;

  // ---- Build / Update ----

  /** Build the skeleton graph from GridWorld's cell connectivity.
   *  阶段 3: 以 merger_graph 的 connected 节点为节点源，聚合到 cell 做骨架化。
   *  - 每个 EXPLORING+COVERED cell 取其 GetMergerGraphNodeIndices() 中最靠近 cell 中心的 connected 节点作为代表
   *  - 节点位置 = merger_graph 节点的实际位置（不是 cell 中心，避免落在障碍物上）
   *  - 边权 = 两代表节点在 merger_graph 上的实际最短路径距离（更准确）
   *  - 自然继承 merger_graph 的多机器人共享信息（间接共享方案 A） */
  void BuildFromGridWorld(
      grid_world_ns::GridWorld& grid_world,
      const merger_graph_ns::MergerGraph& merger_graph);

  /** Incremental update: add new cells / update connectivity.
   *  阶段 2.1 + 3.3b: 基于 added/removed cell 集合做增量更新，不全量重建。
   *  阶段 3: 增量更新也需要 merger_graph 参数（新增 cell 的代表点需从 merger_graph 选取）。
   *  - 只有 added: 增量添加节点和边
   *  - 有 removed: 降级为全量重建（删除节点索引重排复杂）
   */
  void UpdateFromGridWorld(grid_world_ns::GridWorld& grid_world,
                           const merger_graph_ns::MergerGraph& merger_graph);

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
   *  If use_cached is true, uses precomputed all-pairs distances (小图)
   *  or Dijkstra + LRU cache (大图).
   *  Returns -1.0 if not available (caller should fallback to merger_graph). */
  double GetShortestPathDist(int from_cell, int to_cell, bool use_cached = true);

  /** Get shortest path as a nav_msgs::msg::Path between two positions.
   *  阶段 3.3a: 先用骨架图 A* 得到 cell 序列，再对每对相邻 cell 的代表节点用
   *  merger_graph->GetShortestPath 细化实际路径，使路径同时享受骨架图的速度
   *  和 merger_graph 的精度。回退到 merger_graph 全程（若骨架图为空或降级）。
   *  Returns false if no path exists. */
  bool GetShortestPath(const geometry_msgs::msg::Point& start,
                       const geometry_msgs::msg::Point& goal,
                       bool get_path,
                       nav_msgs::msg::Path& path,
                       const merger_graph_ns::MergerGraph& merger_graph);

  /** Get all pairs shortest path matrix (小图 Floyd-Warshall 预计算; 大图为空). */
  const std::vector<std::vector<double>>& GetAllPairsDist() const {
    return all_pairs_dist_;
  }

  // ---- Stats ----

  int GetNodeNum() const { return nodes_.size(); }
  int GetEdgeNum() const;
  bool IsDirty() const { return dirty_; }

  /** 是否处于降级模式（节点数超限或大图模式）。 */
  bool IsDegraded() const { return degraded_; }

  /** 阶段 2.4: 获取性能统计（命中率、build/update/query 时间等）。 */
  struct Stats {
    int node_num = 0;
    int edge_num = 0;
    int build_count = 0;          // 全量重建次数
    int incremental_update_count = 0;  // 增量更新次数
    int dist_query_count = 0;     // GetShortestPathDist 调用次数
    int dist_query_hit_count = 0; // 成功命中（非 -1）次数
    int dijkstra_query_count = 0; // 大图 Dijkstra 触发次数
    double last_build_ms = 0.0;
    double last_update_ms = 0.0;
    double total_build_ms = 0.0;
    double total_update_ms = 0.0;
    double total_query_ms = 0.0;
  };
  const Stats& GetStats() const { return stats_; }
  void ResetStats();

  /** Enable/disable skeleton graph usage. */
  static bool enabled_;

  /** Get visualization markers for RViz.
   *  Safe to call when graph is empty (returns empty markers). */
  void GetMarker(visualization_msgs::msg::Marker& node_marker,
                 visualization_msgs::msg::Marker& edge_marker) const;

  // ---- 阶段 2: 可配置参数（运行时设定） ----
  //   默认值见 .cpp 顶部；可由外部 setter 修改（例如从 yaml 参数注入）
  void SetMaxFloydWarshallNodeNum(int n) { kMaxFloydWarshallNodeNum_ = n; }
  void SetMaxSkeletonNodeNum(int n) { kMaxSkeletonNodeNum_ = n; }
  void SetDijkstraCacheSize(int n) { kDijkstraCacheSize_ = n; }

 private:
  void RebuildKDTree();
  void ComputeAllPairsShortestPaths();
  void BuildFromScratch(grid_world_ns::GridWorld& grid_world,
                        const merger_graph_ns::MergerGraph& merger_graph);

  // 阶段 2.1 + 3.3b: 增量更新辅助函数（需要 merger_graph 选取代表点）
  void AddCellNode(grid_world_ns::GridWorld& grid_world,
                   const merger_graph_ns::MergerGraph& merger_graph,
                   int cell_ind);
  void AddEdgesForCell(grid_world_ns::GridWorld& grid_world, int cell_ind);
  bool IncrementalUpdate(grid_world_ns::GridWorld& grid_world,
                         const merger_graph_ns::MergerGraph& merger_graph,
                         const std::vector<int>& added_cells,
                         const std::vector<int>& removed_cells);

  // 阶段 3.2: 骨架化——度数过滤，移除非边界叶子节点
  void PruneLeafNodes(grid_world_ns::GridWorld& grid_world);

  // 阶段 3.1: 从 cell 的 GetMergerGraphNodeIndices() 中选最靠近 cell 中心的 connected 节点
  //   返回 merger_graph 节点索引；无合适节点返回 -1
  int SelectRepresentativeMergerNode(grid_world_ns::GridWorld& grid_world,
                                     const merger_graph_ns::MergerGraph& merger_graph,
                                     int cell_ind);

  // 阶段 2.3: 大图按需 Dijkstra
  //   从 source 跑 Dijkstra，返回到所有节点的距离向量；命中缓存则直接返回
  std::vector<double> DijkstraFromNode(int source_node);
  //   LRU 缓存: source_node -> (dist_vector, last_use_tick)
  struct DijkstraCacheEntry {
    std::vector<double> dist;
    int64_t last_use_tick = 0;
  };
  std::unordered_map<int, DijkstraCacheEntry> dijkstra_cache_;
  int64_t dijkstra_tick_ = 0;  // 全局 tick，用于 LRU

  // 阶段 2.2: 邻接集用于 O(1) 去重
  //   graph_[i] 是邻接列表（带顺序），graph_set_[i] 是邻接集（快速查找）
  //   两者保持同步：添加边时同时 push_back 和 insert
  std::vector<std::unordered_set<int>> graph_set_;

  std::vector<SkeletonNode> nodes_;
  std::vector<std::vector<int>> graph_;         // adjacency list
  std::vector<std::vector<double>> edge_dist_;  // edge weights

  // index maps
  std::unordered_map<int, int> cell_to_node_;   // cell_index -> skeleton node index

  // KD-tree for fast nearest-neighbour
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr kdtree_cloud_;

  // Cached all-pairs shortest paths (Floyd-Warshall)
  //   阶段 2.3: 仅当 V <= kMaxFloydWarshallNodeNum_ 时计算；大图为空，用 Dijkstra
  std::vector<std::vector<double>> all_pairs_dist_;
  bool all_pairs_dirty_ = true;
  bool dirty_ = true;
  bool degraded_ = false;  // 阶段 2.5: 降级标志

  // Build tracking
  int last_cell_count_ = 0;  // used to detect when full rebuild is needed
  std::unordered_set<int> last_cell_set_;  // 修复 1.4: 记录上次构建的 cell 集合，用于检测集合变化

  // 阶段 2: 可配置阈值
  int kMaxFloydWarshallNodeNum_ = 300;  // V 超过此值则改用 Dijkstra
  int kMaxSkeletonNodeNum_ = 1000;      // V 超过此值则降级（全回退 merger_graph）
  int kDijkstraCacheSize_ = 16;          // Dijkstra LRU 缓存条目数

  // 阶段 2.4: 性能统计
  Stats stats_;
};

}  // namespace skeleton_graph_ns
