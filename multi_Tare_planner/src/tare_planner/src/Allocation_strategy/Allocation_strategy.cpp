#include "../../include/Allocation_strategy/Allocation_strategy.h"

namespace Allocation_strategy_ns
{

//-----------------------------------------------Greedy----------------------------------------------------------//
void GreedySolver::GreedySolver_init(DataModel& data) 
{
    //根据  depot_M  重新构造距离矩阵  distance_matrix_robot_goal 
    //data.depot_M     按顺序排列的   起点、终点
    //data.depot_M[0].first       是 当前机器人的起始点和终点  结点索引   0号其机器人是当前机器人
    //data.depot_M[1].first       是1号机器人的起始点和终点  在距离矩阵上的编号  需要重新排列  编号,并在最后返回

    //提取机器人  索引
    std::unordered_set<int>robot_index;
    for(int i=0;i<data.depot_M.size();i++)
    {
        robot_index.insert(data.depot_M[i].first); // 第一个就是我们需要知道的
        robot_map.insert(std::pair<int,int>(i,data.depot_M[i].first));
    }

    current_robot_idx = 0;
    if (!data.robot_ids.empty())
    {
        for (int i = 0; i < data.robot_ids.size(); i++)
        {
            if (data.robot_ids[i] == data.current_robot_id)
            {
                current_robot_idx = i;
                break;
            }
        }
    }

    for(int i= 0;i<data.distance_matrix.size();i++)  //行点   判定当前点是否是  机器人结点
    {
        if(robot_index.find(i)==robot_index.end())//不是机器人结点  ,目标结点
        {
            int num=goal_map.size();
            goal_map.insert(std::pair<int,int>(num,i));
        }
    }

    for(int i=0;i<robot_map.size();i++)
    {
        int i_temp=robot_map[i];
        std::vector<int> temp;
        for(int j=0;j<goal_map.size();j++)
        {
            int j_temp=goal_map[j];
            temp.push_back(data.distance_matrix[i_temp][j_temp]);
        }
        distance_matrix_robot_goal.push_back( temp);
    }
    return ;
}

void GreedySolver::Solve()
{
    //  贪婪  求解 
    //  查找  距离矩阵里面最小的固定  ,然后  剔除,直到最小的哪个是 当前机器人  则跳出.
    std::unordered_set<int>Assigned_robot_index; //已分配的索引-机器人
    std::unordered_set<int>Assigned_goal_index; //已分配的 索引-目标点
    std::unordered_map<int,int>Assigned_index;  //已分配
    //循环判定  
    //  跳出循环的 要求
    // 1 直到 当前的机器人分配到目标点  (一般在目标点数量>=机器人数量)
    // 2  目标点分配完毕   (没有目标点分配给  机器人了  ,   一般 目标点数量  小于  机器人数量   且  机器人离发现的目标点较远)
    while(!(Assigned_robot_index.find(current_robot_idx)!=Assigned_robot_index.end()  || Assigned_goal_index.size()==goal_map.size()))  // 当前机器人分配到目标即跳出
    {
         int  min_i=-1;
         int  min_j=-1;  //每次循环的最小  min_i;min_j
         int  min_v=99999; //初始化给一个较大的值
        //两层循环吧
        for(int i=0;i<robot_map.size();i++)
        {
            if(Assigned_robot_index.find(i)!=Assigned_robot_index.end())  //已经分配的跳过
            {
                continue;
            }
            for(int j=0;j<goal_map.size();j++)
            {
                if(Assigned_goal_index.find(j)!=Assigned_goal_index.end())  //已分配过的目标点  跳过
                {
                     continue;
                }
                int min_temp=distance_matrix_robot_goal[i][j];    //第i个机器人和目标点的距离矩阵
                if(min_temp<min_v)
                {
                    min_v = min_temp;
                    min_i=i;
                    min_j=j;
                }
            }
        }
        //两层循环出来  将找到的最小值放入  已分配表
        if(min_i!=-1)
        {
            Assigned_robot_index.insert(min_i);
            Assigned_goal_index.insert(min_j);
            Assigned_index.insert(std::pair<int,int>(min_i,min_j));
        }
    }
        if(Assigned_robot_index.find(current_robot_idx)!=Assigned_robot_index.end())  //这是有分配的跳出
    {
            int temp=Assigned_index[current_robot_idx];
        this_index=goal_map[temp];
    }
    else  //其他点分配完,没有分配的跳出     置为  -1
    {
        this_index=-1;
    }

    return ;


}

void GreedySolver::getSolutionNodeIndex(std::vector<int>& node_index,bool has_dummy)
{
    if(this_index!=-1)
    {
        node_index.push_back(this_index);
    }else
    {
        node_index.clear();  //没有分配成功就  清空
    }
    return;
}

//-----------------------------------------------MinPos----------------------------------------------------------//
void MinPosSolver::MinPosSolver_init(DataModel& data)
{
    //根据  depot_M  重新构造距离矩阵  distance_matrix_robot_goal 
    //data.depot_M     按顺序排列的   
    //data.depot_M[0].first       是 当前机器人的起始点和终点  结点索引   0号其机器人是当前机器人
    //data.depot_M[1].first       是1号机器人的起始点和终点  在距离矩阵上的编号  需要重新排列  编号,并在最后返回

    //提取机器人  索引
    std::unordered_set<int>robot_index;
    for(int i=0;i<data.depot_M.size();i++)
    {
        robot_index.insert(data.depot_M[i].first);
        robot_map.insert(std::pair<int,int>(i,data.depot_M[i].first));
    }

    current_robot_idx = 0;
    if (!data.robot_ids.empty())
    {
        for (int i = 0; i < data.robot_ids.size(); i++)
        {
            if (data.robot_ids[i] == data.current_robot_id)
            {
                current_robot_idx = i;
                break;
            }
        }
    }
    //构造索引  map  映射
    for(int i= 0;i<data.distance_matrix.size();i++)  //行点   判定当前点是否是  机器人结点
    {
        if(robot_index.find(i)==robot_index.end())//不是机器人结点  ,目标结点
        {
            int num=goal_map.size();
            goal_map.insert(std::pair<int,int>(num,i));
        }

    }

    //构造  当前的  距离矩阵  distance_matrix_robot_goal 
    for(int i=0;i<robot_map.size();i++)
    {
        int i_temp=robot_map[i];
        std::vector<int> temp;
        for(int j=0;j<goal_map.size();j++)
        {
            int j_temp=goal_map[j];
            temp.push_back(data.distance_matrix[i_temp][j_temp]);
        }
        distance_matrix_robot_goal.push_back( temp);
    }

    //构造  P_distance_matrix_robot_goal 
    for(int i=0;i<robot_map.size();i++)
    {
        std::vector<int>temp;
        for(int j=0;j<goal_map.size();j++)
        {
            int P_ij=0;  //初始化为0
            //求值
            P_ij=GetPValue(i,j);
            temp.push_back(P_ij);
        }
        P_distance_matrix_robot_goal.push_back(temp);

    }

    return ;
}

int  MinPosSolver::GetPValue(int& this_i,int& this_j)
{
    int P=0;
    int C_ij=distance_matrix_robot_goal[this_i][this_j];
    for(int k=0;k<robot_map.size();k++)
    {
        int C_kj=distance_matrix_robot_goal[k][this_j];
        if(k!=this_i && C_kj<C_ij)
        {
            P++;
        }
    }
    return P;
}

void MinPosSolver::Solve()
{
    //  贪婪  求解 
    //  查找  距离矩阵里面最小的固定  ,然后  剔除,直到最小的哪个是 当前机器人  则跳出.
    std::unordered_set<int>Assigned_robot_index; //已分配的索引
    std::unordered_set<int>Assigned_goal_index; //已分配的 索引
    std::unordered_map<int,int>Assigned_index;  //已分配
    //循环判定
    //  跳出循环的 要求
    // 1 直到 当前的机器人分配到目标点  (一般在目标点数量>=机器人数量)
    // 2  目标点分配完毕   (没有目标点分配给  机器人了  ,   一般 目标点数量  小于  机器人数量   且  机器人离发现的目标点较远)
    while(!(Assigned_robot_index.find(current_robot_idx)!=Assigned_robot_index.end()  || Assigned_goal_index.size()==goal_map.size()))  // 当前机器人分配到目标即跳出
    {
         int  min_i=-1;
         int  min_j=-1;  //每次循环的最小  min_i;min_j
         int  min_v=99999; //初始化给一个较大的值
         int min_dis=0;
        //两层循环吧
        for(int i=0;i<robot_map.size();i++)
        {
            if(Assigned_robot_index.find(i)!=Assigned_robot_index.end())  //已经分配的跳过
            {
                continue;
            }
            for(int j=0;j<goal_map.size();j++)
            {
                if(Assigned_goal_index.find(j)!=Assigned_goal_index.end())  //已分配过的目标点  跳过
                {
                     continue;
                }
                int min_temp=P_distance_matrix_robot_goal[i][j];
                if(min_temp<min_v  )  
                {
                    min_i=i;
                    min_j=j;
                    min_v = min_temp;
                    min_dis=distance_matrix_robot_goal[min_i][min_j];
                }
                if(min_temp==min_v)  //如果相等  就采用   距离近的那个
                {
                    int min_dis_temp=distance_matrix_robot_goal[i][j];
                    if(min_dis_temp<min_dis)
                    {
                        min_i=i;
                        min_j=j;
                        min_dis=min_dis_temp;
                    }
                }
            }
        }
        //两层循环出来  将找到的最小值放入  已分配表
        if(min_i!=-1)
        {
            Assigned_robot_index.insert(min_i);
            Assigned_goal_index.insert(min_j);
            Assigned_index.insert(std::pair<int,int>(min_i,min_j));
        }
    }

   if(Assigned_robot_index.find(current_robot_idx)!=Assigned_robot_index.end())  //这是有分配的跳出
    {
       int temp=Assigned_index[current_robot_idx];
        this_index=goal_map[temp];
    }
    else  //其他点分配完,没有分配的跳出     置为  -1
    {
        this_index=-1;
    }

    return ;


}

void MinPosSolver::getSolutionNodeIndex(std::vector<int>& node_index,bool has_dummy)
{
     if(this_index!=-1)
    {
        node_index.push_back(this_index);
    }else
    {
        node_index.clear();  //没有分配成功就  清空
    }
    return;
}

}
