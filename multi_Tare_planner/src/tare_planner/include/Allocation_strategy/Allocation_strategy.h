// #ifndef   Allocation_strategy_H
// #define  Allocation_strategy_H
#include<cmath>
#include<vector> 
#include <unordered_map> 
#include <unordered_set> 
#include <iostream> 
#include <algorithm> 

namespace Allocation_strategy_ns
{
    struct DataModel;
    class GreedySolver;  //贪婪
    class MinPosSolver;  //位置优先级
}
struct Allocation_strategy_ns::DataModel
{
    std::vector<std::vector<int>> distance_matrix;  //邻接矩阵的距离矩阵
    // int num_vehicles = 1;  //默认为1  可以修改
    // bool is_MTSP=false;
    // int depot = 1;  //  机器人起点 终点位置    范围是num_nodes=  distance_matrix.size()        [1,num_nodes];  不是 传统的  [0,num_nodes-1]
    std::vector<std::pair<int, int>> depot_M;   //多个   起点，终点
    DataModel()
    {
        // num_vehicles=1;
        // is_MTSP=false;
        // depot=1;
    }

};

class Allocation_strategy_ns::GreedySolver  //贪婪算法
{
    public:
    GreedySolver(){

    }
    void GreedySolver_init(DataModel& data); //构造函数
    ~GreedySolver()=default;
    void Solve();//求解函数
    int getComputationTime();  //得到计算时间
    void getSolutionNodeIndex(std::vector<int>& node_index,bool has_dummy);    //结果注意本次的结果只有一个
    //参数
    std::vector<std::vector<int>>distance_matrix_robot_goal; //将传入的  DataModel  里的距离矩阵数据  改写成  行是  机器人index  列是 目标index
    std::unordered_map<int,int> robot_map;  //第一个是  机器人的  编号(i)  ,后一个是  机器人在距离矩阵上  的索引
    std::unordered_map<int,int> goal_map;  //第一个是  目标的编号(j) ,后一个是  目标  在距离矩阵上的索引
    int this_index;  //当前机器人  分配的目标点  在  DataModel 的距离矩阵  上的索引

};


class Allocation_strategy_ns::MinPosSolver
{
    public:
    MinPosSolver(){

    }
    void MinPosSolver_init(DataModel& data); //构造函数
    ~MinPosSolver()=default;
    void Solve();//求解函数
    int getComputationTime();
    void getSolutionNodeIndex(std::vector<int>& node_index,bool has_dummy);    //结果注意本次的结果只有一个
    int GetPValue(int& this_i,int& this_j);
    //参数
    std::vector<std::vector<int>>distance_matrix_robot_goal; //将传入的  DataModel  里的距离矩阵数据  改写成  行是  机器人index  列是 目标index
    std::vector<std::vector<int>>P_distance_matrix_robot_goal; //  更新的    pij
    std::unordered_map<int,int> robot_map;  //第一个是  机器人的  编号(i)  ,后一个是  机器人在距离矩阵上  的索引
    std::unordered_map<int,int> goal_map;  //第一个是  目标的编号(j) ,后一个是  目标  在距离矩阵上的索引
    int this_index;  //当前机器人  分配的目标点  在  DataModel 的距离矩阵  上的索引

};





// #endif