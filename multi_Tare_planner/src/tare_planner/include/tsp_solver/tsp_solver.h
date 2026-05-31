//
// Created by caochao on 7/12/19.
//

#ifndef VISUAL_COVERAGE_PLANNER_TSP_SOLVER_H
#define VISUAL_COVERAGE_PLANNER_TSP_SOLVER_H

#include "../Allocation_strategy/Allocation_strategy.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"
#include <cmath>
#include <vector>

using namespace operations_research;

namespace tsp_solver_ns {
struct DataModel;
struct DataModel_M;
class TSPSolver;
} // namespace tsp_solver_ns

struct tsp_solver_ns::DataModel {
  std::vector<std::vector<int>> distance_matrix;     // 邻接矩阵的距离矩阵

  int num_vehicles = 1;  // 机器人数量

  bool is_MTSP = false;   // 是否为多机器人

  std::string Allocation_strategy = "Mdvrp";  // 探索策略

  RoutingIndexManager::NodeIndex depot{0};    // 机器人起点

  std::vector<std::pair<RoutingIndexManager::NodeIndex, RoutingIndexManager::NodeIndex> > depot_M;   // 多个机器人起点的组合

  std::vector<std::pair<int,int>> depot_M_other_strategy;   // 其他策略

  std::vector<int> robot_ids;  // 与 depot_M 对齐的机器人 ID 列表
  int current_robot_id = 0;    // 当前机器人 ID
};

class tsp_solver_ns::TSPSolver {
private:
  DataModel data_;
  // RoutingIndexManager manager_;
  // RoutingModel routing_;

  std::unique_ptr<RoutingIndexManager> manager_;
  std::unique_ptr<RoutingModel> routing_;

  const Assignment *solution_;

public:
  TSPSolver(DataModel data);
  ~TSPSolver() = default;

  void Solve();

  int getComputationTime();

  void getSolutionNodeIndex(std::vector<int> &node_index, bool has_dummy);

  double getPathLength();

  bool is_MTSP; // 多机器人

  int TSP_depot; //当前机器人起点
  std::string Allocation_strategy ; //外部传入   探索  策略  

  Allocation_strategy_ns::DataModel DataModel_other;

  Allocation_strategy_ns::GreedySolver Greedy_Solver;  //贪婪策略
  Allocation_strategy_ns::MinPosSolver MinPos_Solver;  //位置优先级
};

#endif // VISUAL_COVERAGE_PLANNER_TSP_SOLVER_H
