#include "../../include/tsp_solver/tsp_solver.h"

namespace tsp_solver_ns {

TSPSolver::TSPSolver(tsp_solver_ns::DataModel data) : data_(std::move(data))
{
  is_MTSP = data_.is_MTSP;
  Allocation_strategy = data_.Allocation_strategy;

  if(!data_.is_MTSP)
  {
    manager_ = std::make_unique<RoutingIndexManager>(data_.distance_matrix.size(), data_.num_vehicles, data_.depot);   // 创建路由索引管理器
    routing_ = std::make_unique<RoutingModel>(*manager_);  // 创建路由模型
  }
  else
  {
    //多机器人分为 MinDis最近邻策略  Greedy贪婪策略  MinPos位置分级策略  Mdvrp 求解多站点车辆路由问题策略（or-tools 求解）
    DataModel_other.depot_M=data_.depot_M_other_strategy;
    DataModel_other.robot_ids = data_.robot_ids;
    DataModel_other.current_robot_id = data_.current_robot_id;
    int current_robot_idx = 0;
    if (!DataModel_other.robot_ids.empty())
    {
      for (int i = 0; i < DataModel_other.robot_ids.size(); i++)
      {
        if (DataModel_other.robot_ids[i] == DataModel_other.current_robot_id)
        {
          current_robot_idx = i;
          break;
        }
      }
    }
    if (current_robot_idx >= 0 && current_robot_idx < DataModel_other.depot_M.size())
    {
      TSP_depot = DataModel_other.depot_M[current_robot_idx].first;
    }
    else
    {
      TSP_depot = DataModel_other.depot_M[0].first;
    }

    if(Allocation_strategy == "MinDis")
    {
        return;  //  多机器人最近邻策略  不做处理，后续自动将最近点作为目标点发布
    }else if(Allocation_strategy == "Greedy")
    {
        DataModel_other.distance_matrix = data_.distance_matrix;
        Greedy_Solver.GreedySolver_init(DataModel_other);
        return;
    }else if(Allocation_strategy == "MinPos")
    {
        DataModel_other.distance_matrix=data_.distance_matrix;
        MinPos_Solver.MinPosSolver_init(DataModel_other);
        return;
    }
    manager_ = std::make_unique<RoutingIndexManager>(data_.distance_matrix.size(), data_.num_vehicles, data_.depot_M);   //MTSP
    routing_ = std::make_unique<RoutingModel>(*manager_);
  }
}

void TSPSolver::Solve() {
  if(is_MTSP  &&  Allocation_strategy == "MinDis")
  {
      return; 
  } else if(  is_MTSP  &&  Allocation_strategy == "Greedy")
  {
      Greedy_Solver.Solve();
      return;
  } else if( is_MTSP  &&  Allocation_strategy == "MinPos")
  {
      MinPos_Solver.Solve();
      return;
  }

  //后续则是  TSP  或者 是  MTSP   (Mdvrp  的求解)
  const int transit_callback_index = routing_->RegisterTransitCallback([this](int64_t from_index, int64_t to_index) -> int64_t {
    auto from_node = manager_->IndexToNode(from_index).value();
    auto to_node = manager_->IndexToNode(to_index).value();
    return data_.distance_matrix[from_node][to_node];
  });

  routing_->SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
  searchParameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);

  solution_ = routing_->SolveWithParameters(searchParameters);
}

int TSPSolver::getComputationTime() { return routing_->solver()->wall_time(); }

void TSPSolver::getSolutionNodeIndex(std::vector<int> &node_index, bool has_dummy) {
  node_index.clear();
  if(is_MTSP && Allocation_strategy == "MinDis") 
  {
    //输出
      node_index.push_back(TSP_depot);//当前位置
      return;
  } else if(is_MTSP && Allocation_strategy == "Greedy")
  {
      //输出
      node_index.push_back(TSP_depot);//当前位置
      std::vector<int> node_index_Greedy;
      Greedy_Solver.getSolutionNodeIndex(node_index_Greedy,true);
      if(!node_index_Greedy.empty())
      {
          node_index.push_back(node_index_Greedy[0]);
          // std::cout<<"Greedy 分配成功"<<std::endl;
      }
      return ;
  }else if(is_MTSP && Allocation_strategy == "MinPos")
  {
      //输出
      node_index.push_back(TSP_depot);//当前位置
      std::vector<int> node_index_MinPos;
      MinPos_Solver.getSolutionNodeIndex(node_index_MinPos,true);
      if(!node_index_MinPos.empty())
      {
          node_index.push_back(node_index_MinPos[0]);
          // std::cout<<"MinPos 分配成功"<<std::endl;
      }
      return ;
  }

  int64_t index = routing_->Start(0);
  int64_t end_index = index;
  while (routing_->IsEnd(index) == false) {
    node_index.push_back(static_cast<int>(manager_->IndexToNode(index).value()));
    index = solution_->Value(routing_->NextVar(index));
  }
  if (has_dummy) {
    int dummy_node_index = data_.distance_matrix.size() - 1;
    if (node_index[1] == dummy_node_index) {
      node_index.erase(node_index.begin() + 1);
      node_index.push_back(node_index[0]);
      node_index.erase(node_index.begin());
      std::reverse(node_index.begin(), node_index.end());
    } else 
    {
      node_index.pop_back();
    }
  }
}

double TSPSolver::getPathLength() {
  return (solution_->ObjectiveValue()) / 10.0;
}

} // namespace tsp_solver_ns
