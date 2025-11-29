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
void PrintSolution(const RoutingIndexManager &manager, const RoutingModel &routing, const Assignment &solution);
} // namespace tsp_solver_ns

struct tsp_solver_ns::DataModel {
  std::vector<std::vector<int>> distance_matrix;
  int num_vehicles = 1;
  bool is_MTSP=false;
  std::string Allocation_strategy="Mdvrp";
  RoutingIndexManager::NodeIndex depot{0};
  std::vector<std::pair<RoutingIndexManager::NodeIndex, RoutingIndexManager::NodeIndex> > depot_M;   //多个
  std::vector<std::pair<int,int>> depot_M_other_strategy;//其他策略
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
  void PrintSolution();
  int getComputationTime();
  void getSolutionNodeIndex(std::vector<int> &node_index, bool has_dummy);
  double getPathLength();

  bool is_MTSP; //多机器人

  //新增  分配相关
  int TSP_depot; //当前机器人起点
  std::string Allocation_strategy ; //外部传入   探索  策略  

  Allocation_strategy_ns::DataModel DataModel_other;

  //添加  多目标, 多机器人的  分配策略
  //贪婪策略
  Allocation_strategy_ns::GreedySolver Greedy_Solver;
  //位置分级
  Allocation_strategy_ns::MinPosSolver MinPos_Solver;
};

#endif // VISUAL_COVERAGE_PLANNER_TSP_SOLVER_H
