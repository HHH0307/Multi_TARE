#include "../../include/tsp_solver/tsp_solver.h"

namespace tsp_solver_ns {
// TSPSolver::TSPSolver(tsp_solver_ns::DataModel data) : data_(std::move(data)), manager_(data_.distance_matrix.size(), data_.num_vehicles, data_.depot), routing_(manager_) 
TSPSolver::TSPSolver(tsp_solver_ns::DataModel data) : data_(std::move(data))
{
  // Create Routing Index Manager
  is_MTSP=data_.is_MTSP;
  Allocation_strategy=data_.Allocation_strategy;
  // std::cout<<Allocation_strategy<<std::endl;
  // std::cout<<data.Allocation_strategy<<std::endl;
  // std::cout<<data_.Allocation_strategy<<std::endl;
  // std::cout<<"is_MTSP  "<<data.is_MTSP<<std::endl;
  // std::cout<<"is_MTSP  "<<data_.is_MTSP<<std::endl;
  // std::cout<<"一次    "<<std::endl;

  if(!data_.is_MTSP)
  {
    manager_ = std::make_unique<RoutingIndexManager>(data_.distance_matrix.size(), data_.num_vehicles, data_.depot);   //TSMP
    // Create Routing Model.
    routing_ = std::make_unique<RoutingModel>(*manager_);
  }
  else
  {
    //多机器人分为 MinDis最近邻策略  Greedy贪婪策略  MinPos位置分级策略    Mdvrp  求解多站点车辆路由问题策略（or-tools 求解）
    DataModel_other.depot_M=data_.depot_M_other_strategy;
    TSP_depot=DataModel_other.depot_M[0].first; 
    if(Allocation_strategy=="MinDis")
    {
        //  最近点在外面有补偿 不做处理  后续自动  将最近点  当作目标点  发布
        return;
    }else if(Allocation_strategy=="Greedy")
    {
        //给数据参数  并  初始化 Greedy
        DataModel_other.distance_matrix=data_.distance_matrix;
        Greedy_Solver.GreedySolver_init(DataModel_other);
        return;
    }else if(Allocation_strategy=="MinPos")
    {
        //给数据参数    并   初始化 MinPos
        DataModel_other.distance_matrix=data_.distance_matrix;
        MinPos_Solver.MinPosSolver_init(DataModel_other);
        return;
    }
    manager_ = std::make_unique<RoutingIndexManager>(data_.distance_matrix.size(), data_.num_vehicles, data_.depot_M);   //MTSP
    // Create Routing Model.
    routing_ = std::make_unique<RoutingModel>(*manager_);
    
  }
}

void TSPSolver::Solve() {
  //由  前面 3种  策略  先判断
  //1  外部的最近点策略
    if(is_MTSP  &&  Allocation_strategy=="MinDis")
    {
        //  最近点在外面有补偿 不做处理  后续自动  将最近点  当作目标点  发布
        return;
    }else if(  is_MTSP  &&  Allocation_strategy=="Greedy") //2  贪婪策略
    {
        //运行求解函数
        Greedy_Solver.Solve();
        return;
    }else if( is_MTSP  &&  Allocation_strategy=="MinPos") //3 MinPos 策略
    {
        //运行求解函数
        MinPos_Solver.Solve();
        return;
    }

    //后续则是  TSP  或者 是  MTSP   (Mdvrp  的求解)

  const int transit_callback_index = 
      routing_->RegisterTransitCallback([this](int64_t from_index, int64_t to_index) -> int64_t {
        // Convert from routing variable Index to distance matrix NodeIndex.
        auto from_node = manager_->IndexToNode(from_index).value();
        auto to_node = manager_->IndexToNode(to_index).value();
        return data_.distance_matrix[from_node][to_node];
      });

  // Define cost of each arc.
  routing_->SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  // Setting first solution heuristic.
  RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
  searchParameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);

  // Solve the problem.
  solution_ = routing_->SolveWithParameters(searchParameters);
}

int TSPSolver::getComputationTime() { return routing_->solver()->wall_time(); }

void TSPSolver::getSolutionNodeIndex(std::vector<int> &node_index,
                                     bool has_dummy) {
  node_index.clear();
  if(is_MTSP && Allocation_strategy == "MinDis")  //1 最近邻策略
  {
    //输出
      node_index.push_back(TSP_depot);//当前位置
      return;
  }else if(is_MTSP && Allocation_strategy == "Greedy") //2  贪婪策略 Greedy
  {
      //输出
      node_index.push_back(TSP_depot);//当前位置
      std::vector<int> node_index_Greedy;
      Greedy_Solver.getSolutionNodeIndex(node_index_Greedy,true);
      if(!node_index_Greedy.empty())
      {
          node_index.push_back(node_index_Greedy[0]);
          std::cout<<"Greedy  分配成功"<<std::endl;
      }
      return ;
  }else if(is_MTSP && Allocation_strategy == "MinPos") //3 位置分级策略 MinPos 
  {
      //输出
      node_index.push_back(TSP_depot);//当前位置
      std::vector<int> node_index_MinPos;
      MinPos_Solver.getSolutionNodeIndex(node_index_MinPos,true);
      if(!node_index_MinPos.empty())
      {
          node_index.push_back(node_index_MinPos[0]);
          std::cout<<"MinPos  分配成功"<<std::endl;
      }
      return ;
  }

  int64_t index = routing_->Start(0);
  int64_t end_index = index;
  while (routing_->IsEnd(index) == false) {
    node_index.push_back(static_cast<int>(manager_->IndexToNode(index).value()));
    index = solution_->Value(routing_->NextVar(index));
  }
  // push back the end node index
  //       node_index.push_back(end_index);
  if (has_dummy) {
    int dummy_node_index = data_.distance_matrix.size() - 1;
    if (node_index[1] == dummy_node_index) {
      // delete dummy node
      node_index.erase(node_index.begin() + 1);
      // push the start node to the end
      node_index.push_back(node_index[0]);
      // remove the start node at the begining
      node_index.erase(node_index.begin());
      // reverse the whole array
      std::reverse(node_index.begin(), node_index.end());
    } else // the last node is dummy node
    {
      node_index.pop_back();
    }
  }
}

double TSPSolver::getPathLength() {
  return (solution_->ObjectiveValue()) / 10.0;
}

} // namespace tsp_solver_ns
