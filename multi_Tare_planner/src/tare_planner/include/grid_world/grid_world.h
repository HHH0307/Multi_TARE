/**
 * @file grid_world.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a grid world
 * @version 0.1
 * @date 2019-11-06
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>
#include <memory>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <grid/grid.h>
#include <tsp_solver/tsp_solver.h>
#include <keypose_graph/keypose_graph.h>

#include "merger_graph/merger_graph.h" // 新增

#include <exploration_path/exploration_path.h>

#include "tare_planner/srv/request_path.hpp" // 新增 服务定义的数据 

#include "tare_planner/msg/subgraph.hpp"  // 新增  话题定义的数据
#include "skeleton_graph/skeleton_graph.h"  //骨架图
#include <unordered_map>
#include <unordered_set>


namespace viewpoint_manager_ns
{
class ViewPointManager; //视点管理
}

namespace grid_world_ns
{
enum class CellStatus
{
  UNSEEN = 0,  //看不见单元   编号为0
  EXPLORING = 1,  //探索单元   编号为1
  COVERED = 2,    //覆盖单元   编号为2
  COVERED_BY_OTHERS = 3,    //被其他覆盖   编号为3
  NOGO = 4   //无法通行  编号为4
};

// 新增 -- 设置机器人状态
enum class RobotStatus
{
  Exploring = 0,  //探索状态   编号为0   自己发布目标点
  Request = 1,  //到达目标点阶段   请求路径   编号为1    自己发布目标点
  Far_planner = 2,    //到达目标点阶段  使用Far规划器  计算路径   编号为2   far-planner 节点发布目标点
  Global_tsp=3,     //全局  TSP
  Return_home=4,   //返回起点
};

//添加   fuse_node
struct fuse_node
{
  int Cell_id_;
  int robot_id_;//  结点来自哪个子图
  int positions_id_;  //  外部传入的   位置容器  索引   用于计算边时便于寻找
  int fuse_grapher_node_id_;   //当前  融合结点  在  融合图上的ID
  bool is_robotpose;  //是否是机器人位置
  bool is_Explored; //是否已经  探索  在keyposegraph 图中
  geometry_msgs::msg::Point position_; //位置数据
  
  explicit fuse_node(int Cell_id, int robot_id,  int positions_id,int fuse_grapher_node_id, geometry_msgs::msg::Point position)
  {
    Cell_id_=Cell_id;
    robot_id_=robot_id;
    positions_id_=positions_id;
    fuse_grapher_node_id_=fuse_grapher_node_id;
    is_robotpose=false;
    is_Explored=false;
    position_=position;
  }
  ~fuse_node() = default;
};

//  添加  fuse_grapher     融合图   结构体
struct fuse_grapher
{
    //融合图  结构体
    std::vector<std::vector<int>> graph_;    //图    矩阵   整数形
    std::vector<std::vector<double>> dist_;   //距离矩阵
    std::vector<fuse_node> fuse_nodes_;   //  融合结点结点     向量
    
    void AddNode(int Cell_id, int  robot_id, int positions_id,int fuse_grapher_node_id,const geometry_msgs::msg::Point& position)
    {
        fuse_node new_node(Cell_id, robot_id, positions_id, fuse_grapher_node_id, position);
        new_node.is_robotpose=false;   //初始化都定义为   不是
        fuse_nodes_.push_back(new_node);
        std::vector<int> neighbors;
        graph_.push_back(neighbors);
        std::vector<double> neighbor_dist;
        dist_.push_back(neighbor_dist);
    }
    
    // 一个函数  即加结点  由加边
    void AddNodeAndEdge(int Cell_id, int  robot_id, int positions_id,int fuse_grapher_node_id,const geometry_msgs::msg::Point& position,
                                      int connected_node_ind, double connected_node_dist)   //  connected_node_ind  前面一个
    {
        AddNode(Cell_id, robot_id, positions_id, fuse_grapher_node_id, position);
        AddEdge(connected_node_ind, fuse_grapher_node_id, connected_node_dist);      //  connected_node_ind  前面一个   node_ind  后面一个   有方向？connected_node_dist边长度
    }

    void AddEdge(int from_node_ind, int to_node_ind, double dist)
    {
        MY_ASSERT(from_node_ind >= 0 && from_node_ind < graph_.size() && from_node_ind < dist_.size());     //断言判断范围是否有错
        MY_ASSERT(to_node_ind >= 0 && to_node_ind < graph_.size() && to_node_ind < dist_.size());

        graph_[from_node_ind].push_back(to_node_ind);  //   边的表示   就是用  邻接矩阵
        graph_[to_node_ind].push_back(from_node_ind);    //graph_ 表示点的连接关系

        dist_[from_node_ind].push_back(dist);   //表示   边长大小
        dist_[to_node_ind].push_back(dist);   //表示   边长大小
    }
    
    bool is_exist_fuse_node(fuse_node& new_node,int& node_id)
    {
        for(int i=0;i<fuse_nodes_.size();i++)
        {
            if(new_node.robot_id_==fuse_nodes_[i].robot_id_ &&
               new_node.positions_id_==fuse_nodes_[i].positions_id_ )
            {
                node_id=fuse_nodes_[i].fuse_grapher_node_id_;
                return true;   //存在
            }
        }
        return false;
    }

    bool is_edge_exist(int& from_node_ind,int& to_node_ind,double& distance)
    {
        for(int i=0;i<graph_[ from_node_ind].size();i++)
        {
            if(graph_[ from_node_ind][i]==to_node_ind)
            {
                if(distance<dist_[ from_node_ind][i])  //存在边  输入距离较小   就替换
                {
                    dist_[ from_node_ind][i]=distance;
                    for(int j=0;j<graph_[ to_node_ind].size();j++)
                    {
                        if(graph_[ to_node_ind][j]==from_node_ind)
                        {
                            dist_[ to_node_ind][j]=distance;
                        }
                    }
                }
                return true;
            }
        }
        return false;
    }

    bool FindNode_id(int& robot_id,int& positions_id,int& fuse_grapher_node_id)
    {
        for(int i=0;i<fuse_nodes_.size();i++)
        {
            if(robot_id==fuse_nodes_[i].robot_id_ &&
               positions_id==fuse_nodes_[i].positions_id_ )
            {
                fuse_grapher_node_id=fuse_nodes_[i].fuse_grapher_node_id_;
                return true;   //存在
            }
        }
        return false;
    }
};

//新增 MTSP_graph_node
struct  MTSP_graph_node
{
    bool is_keypose_;//是否是机器人历史位置形成的结点。  true  机器人历史位置，false  网格点
    int robot_id_;  // 当前结点是属于哪个机器人产生的  本地网格 和本地机器人位置  都是本地产生，通过外部参数获得。
    int cell_id_; //当前结点是属于哪个     单元网格的
    geometry_msgs::msg::Point position_; //位置数据
    std::vector<int> blacklist_node_index;  //  结点索引   黑名单   及对于额外添加边的时候，在黑名单上的结点不予相连  
    bool is_connected_;//连接到主通道
    int node_index_;//结点编号
    
    MTSP_graph_node()
    {
        cell_id_=-1;
        robot_id_=-1;
        position_.x= -1;
        position_.y= -1;
        position_.z= -1;
        is_keypose_= false;
        node_index_= -1;
        is_connected_=false;
    }
    
    explicit MTSP_graph_node(int robot_id, int cell_id, geometry_msgs::msg::Point position,bool is_keypose,int node_index)
    {
        cell_id_=cell_id;
        robot_id_=robot_id;
        position_.x= position.x;
        position_.y= position.y;
        position_.z= position.z;
        is_keypose_=is_keypose;
        node_index_=node_index;
        is_connected_=false;
    }
    
    ~MTSP_graph_node() = default;
};

//   新增 MTSP_grid_graph
class MTSP_grid_graph
{
public:
    explicit MTSP_grid_graph();//构造函数
    ~ MTSP_grid_graph()= default;;
    
    //成员   关键存储
    std::vector<std::vector<int>> graph_;    //图    矩阵   整数形
    std::vector<std::vector<double>> dist_;   //距离矩阵
    std::vector<MTSP_graph_node> MTSP_graph_nodes_;   //  融合结点结点     向量
    
    //用于查询
    std::unordered_map<int,std::unordered_map<int,int>> find_map_;//查询  map,  机器人id_cell_id 在   MTSP_graph_nodes_   上的地址
    std::unordered_map<int,geometry_msgs::msg::Point> last_robot_position_;
    std::unordered_map<int,int> last_robot_node_index_;

    std::vector<int> connected_node_indices_;   //连接的  结点  ID

    //两边 正向反向查询    通过地址 查询 MTSP_graph_node  里面的数据
    //对于图结点的操作      只  删除边关系，不删除点，点不影响

    //  相关计算切换数据结构  计算加速
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_connected_nodes_;   //用  kdtree   连接结点
    pcl::PointCloud<pcl::PointXYZI>::Ptr connected_nodes_cloud_;    //连接结点云
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_nodes_;   //kdtree   结点
    pcl::PointCloud<pcl::PointXYZI>::Ptr nodes_cloud_;   //结点 云


    //成员函数
    //添加点
    void AddNode( int robot_id, int cell_id, geometry_msgs::msg::Point position,bool is_keypose,int node_index)
    {
        MTSP_graph_node new_node( robot_id,   cell_id,  position,is_keypose,node_index);
        MTSP_graph_nodes_.push_back(new_node);
        std::vector<int> neighbors;
        graph_.push_back(neighbors);
        std::vector<double> neighbor_dist;
        dist_.push_back(neighbor_dist);
    }
    
    //添加边
    void AddEdge(int from_node_ind, int to_node_ind, double dist)
    {
        MY_ASSERT(from_node_ind >= 0 && from_node_ind < graph_.size() && from_node_ind < dist_.size());     //断言判断范围是否有错
        MY_ASSERT(to_node_ind >= 0 && to_node_ind < graph_.size() && to_node_ind < dist_.size());

        graph_[from_node_ind].push_back(to_node_ind);  //   边的表示   就是用  邻接矩阵
        graph_[to_node_ind].push_back(from_node_ind);    //graph_ 表示点的连接关系

        dist_[from_node_ind].push_back(dist);   //表示   边长大小
        dist_[to_node_ind].push_back(dist);   //表示   边长大小
    }

    //删除边
    void DelEdge(int from_node_ind, int to_node_ind)
    {
        MY_ASSERT(from_node_ind >= 0 && from_node_ind < graph_.size() && from_node_ind < dist_.size());     //断言判断范围是否有错
        MY_ASSERT(to_node_ind >= 0 && to_node_ind < graph_.size() && to_node_ind < dist_.size());
        //  from_node_ind  的边容器中  删除  to_node_ind
        for (int k = 0; k < graph_[from_node_ind].size(); k++)
        {
            if (graph_[from_node_ind][k] == to_node_ind)
            {
                graph_[from_node_ind].erase(graph_[from_node_ind].begin() + k);
                dist_[from_node_ind].erase(dist_[from_node_ind].begin() + k);
                k--;
            }
        }
        //  to_node_ind    的边容器中  删除  from_node_ind
        for (int k = 0; k < graph_[to_node_ind].size(); k++)
        {
            if (graph_[to_node_ind][k] == from_node_ind)
            {
                graph_[to_node_ind].erase(graph_[to_node_ind].begin() + k);
                dist_[to_node_ind].erase(dist_[to_node_ind].begin() + k);
                k--;
            }
        }
    }

    //检测边是否存在
    bool is_edge_exist(int& from_node_ind,int& to_node_ind)
    {
        MY_ASSERT(from_node_ind >= 0 && from_node_ind < graph_.size() && from_node_ind < dist_.size());     //断言判断范围是否有错
        MY_ASSERT(to_node_ind >= 0 && to_node_ind < graph_.size() && to_node_ind < dist_.size());
        for(int i=0;i<graph_[ from_node_ind].size();i++)
        {
            if(graph_[ from_node_ind][i]==to_node_ind)
            {
                return true;
            }
        }
        return false;
    }

    //更新边
    void UpdateEdge(int& from_node_ind,int& to_node_ind,double& distance)
    {
        MY_ASSERT(from_node_ind >= 0 && from_node_ind < graph_.size() && from_node_ind < dist_.size());     //断言判断范围是否有错
        MY_ASSERT(to_node_ind >= 0 && to_node_ind < graph_.size() && to_node_ind < dist_.size());
        for(int i=0;i<graph_[ from_node_ind].size();i++)
        {
            if(graph_[from_node_ind][i]==to_node_ind)
            {
                dist_[from_node_ind][i]=distance;
            }
        }
        for(int i=0;i<graph_[to_node_ind].size();i++)
        {
            if(graph_[to_node_ind][i] == from_node_ind)
            {
                dist_[to_node_ind][i] = distance;
            }
        }
    }

    //更新位置
    void GetConnectedNodeIndices(int query_ind, std::vector<int>& connected_node_indices,
                                           std::vector<bool> constraints);
    void GetClosestNodeIndAndDistance(const geometry_msgs::msg::Point& point, int& node_ind, double& dist);
    void UpdateNodes();

    void GetClosestConnectedNodeIndAndDistance(const geometry_msgs::msg::Point& point, int& node_ind, double& dist);
};

class Cell    //网格中的单元    类 
{
public:
    explicit Cell(double x = 0.0, double y = 0.0, double z = 0.0);  //构造函数
    explicit Cell(const geometry_msgs::msg::Point& center);
    ~Cell() = default;
    
    bool IsCellConnected(int cell_ind)  // O(1) 通过set查找
    {
        return connected_cell_set_.find(cell_ind) != connected_cell_set_.end();
    }
    void AddViewPoint(int viewpoint_ind)    //  增加 视点
    {
        viewpoint_indices_.push_back(viewpoint_ind);  //放入视点容器里面
    }
    
    void AddGraphNode(int node_ind)   //增加图结点ID 到容器    keypose_graph_node_indices_
    {
        keypose_graph_node_indices_.push_back(node_ind);
    }
    
    void AddConnectedCell(int cell_ind)  //增加连接的单元ID
    {
        connected_cell_indices_.push_back(cell_ind);
        connected_cell_set_.insert(cell_ind);
        misc_utils_ns::UniquifyIntVector(connected_cell_indices_);
    }

    //新增   long_term_connected_cell_indices_   增加函数
    void AddLongTermConnectedCell(int cell_ind)  //增加连接的单元ID
    {
        long_term_connected_cell_indices_.push_back(cell_ind);
        //去掉相同的
        misc_utils_ns::UniquifyIntVector(long_term_connected_cell_indices_);
    }

    void ClearViewPointIndices()
    {
        viewpoint_indices_.clear();
    }
    
    void ClearGraphNodeIndices()
    {
        keypose_graph_node_indices_.clear();
    }
    
    void ClearConnectedCellIndices()
    {
        connected_cell_indices_.clear();
        connected_cell_set_.clear();
    }

    //新增  long_term_connected_cell_indices_  的清空函数
    void ClearLongTermConnectedCellIndices()
    {
        long_term_connected_cell_indices_ .clear();
    }
    
    //新增  删除  某个点  函数 
    void DelOneCellLongTermConnectedCellIndices(int cell_index)
    {
        //去掉相同的点
        misc_utils_ns::UniquifyIntVector(long_term_connected_cell_indices_);
        std::vector<int>::iterator  index_=std::find(long_term_connected_cell_indices_.begin(),long_term_connected_cell_indices_.end(),cell_index);
        if(index_!=long_term_connected_cell_indices_.end())
        {
            long_term_connected_cell_indices_.erase(index_);
        }
    }

    CellStatus GetStatus()   //返回单元状态
    {
        return status_;
    }
    
    int GetStatusInt()
    {
        if(status_==CellStatus::UNSEEN)
        {return 0;}
        else if(status_==CellStatus::EXPLORING)
        {return 1;}
        else if(status_==CellStatus::COVERED)
        {return 2;}
        else if(status_==CellStatus::COVERED_BY_OTHERS)
        {return 3;}
        else if(status_==CellStatus::NOGO)
        {return 4;}
        return 0;
    }
    
    void SetStatus(CellStatus status) //设置单元状态
    {
        status_ = status;
    }
    
    std::vector<int> GetViewPointIndices() //得到视点ID
    {
        return viewpoint_indices_;
    }
    
    std::vector<int> GetConnectedCellIndices()//得到连接单元ID
    {
        return connected_cell_indices_;
    }

    //新增  long_term_connected_cell_indices_  的相关函数
    std::vector<int> GetLongTermConnectedCellIndices()//得到连接单元ID
    {
        return  long_term_connected_cell_indices_;
    }

    std::vector<int> GetGraphNodeIndices()//得到图结点ID
    {
        return keypose_graph_node_indices_;
    }
    
    geometry_msgs::msg::Point GetPosition() //得到位置
    {
        return center_;
    }
    
    void SetPosition(const geometry_msgs::msg::Point& position)  //设置此单元的位置
    {
        center_ = position;
    }
    
    void SetRobotPosition(const geometry_msgs::msg::Point& robot_position)  //设置机器人位姿
    {
        robot_position_ = robot_position;
        robot_position_set_ = true;
    }
    
    void SetKeyposeID(int keypose_id)//设置关键位姿ID
    {
        keypose_id_ = keypose_id;
        robot_position_set_ = true;
    }
    
    bool IsRobotPositionSet()  //是否已为此单元设置机器人位置
    {
        return robot_position_set_;
    }
    
    geometry_msgs::msg::Point GetRobotPosition()  //得到机器人位置
    {
        return robot_position_;
    }
    
    void AddVisitCount()  //增加访问次数
    {
        visit_count_++;
    }
    
    int GetVisitCount()  //得到访问次数
    {
        return visit_count_;
    }
    
    void Reset();   //重置
    
    int GetKeyposeID()
    {
        return keypose_id_;
    }
    
    void SetViewPointPosition(const Eigen::Vector3d& position)
    {
        viewpoint_position_ = position;
    }
    
    Eigen::Vector3d GetViewPointPosition()
    {
        return viewpoint_position_;
    }
    
    void SetRoadmapConnectionPoint(const Eigen::Vector3d& roadmap_connection_point)
    {
        roadmap_connection_point_ = roadmap_connection_point;
    }
    
    Eigen::Vector3d GetRoadmapConnectionPoint()
    {
        return roadmap_connection_point_;
    }
    
    nav_msgs::msg::Path GetPathToKeyposeGraph()
    {
        return path_to_keypose_graph_;
    }
    
    void SetPathToKeyposeGraph(const nav_msgs::msg::Path& path)
    {
        path_to_keypose_graph_ = path;
    }
    
    bool IsPathAddedToKeyposeGraph()
    {
        return path_added_to_keypose_graph_;
    }
    
    void SetPathAddedToKeyposeGraph(bool add_path)
    {
        path_added_to_keypose_graph_ = add_path;
    }
    
    bool IsRoadmapConnectionPointSet()
    {
        return roadmap_connection_point_set_;
    }
    
    void SetRoadmapConnectionPointSet(bool set)
    {
        roadmap_connection_point_set_ = set;
    }

    int GetMTSP_graph_index_find_exploring_cell_()
    {
        return MTSP_graph_index_find_exploring_cell_;
    }

    void SetMTSP_graph_index_find_exploring_cell_(int index)
    {
        MTSP_graph_index_find_exploring_cell_ = index;
    }

    // 新增  获取  当前网格内的  拼接图    的节点的  索引
    std::vector<int> GetMergerGraphNodeIndices()
    {
        return merger_graph_node_indices_;
    }
    
    // 新增  添加  拼接图节点  到  merger_graph_node_indices_
    void AddNodeIndex2MergerGraphNodeIndices(int merger_grapher_node_index)
    {
        merger_graph_node_indices_.push_back(merger_grapher_node_index);
    }
    
    // 获取  merger_graph_robot_id_set_
    std::set<int> & SetRobotIdSet()
    {
        return merger_graph_robot_id_set_;
    }
    
    //  获取  
    std::set<int> GetRobotIdSet()
    {
        return merger_graph_robot_id_set_;
    }
    
    // 插入数据   到  robotmin_robotmax_isconnected_   按     小  大  排序  插入
    void  InsterData2RMRMIC(int robot_1,int robot_2,bool is_connected=false)
    {
        if(robot_1>robot_2)
        {
            robotmin_robotmax_isconnected_[robot_2][robot_1]=is_connected;
        }else if(robot_1<robot_2)
        {
            robotmin_robotmax_isconnected_[robot_1][robot_2]=is_connected;
        }
    }
    
    // 获取数据
    std::map<int, std::map<int, bool>> GetRMRMIC()
    {
        return robotmin_robotmax_isconnected_;
    }

private:
    CellStatus status_;   //单元状态
    // The center location of this cell.  单元中心位置
    geometry_msgs::msg::Point center_;
    // Position of the robot where this cell is first observed and turned EXPLORING   机器人位置  是单元里面第一次观察和探索
    geometry_msgs::msg::Point robot_position_;
    // Whether the robot position has been set for this cell   是否已为此单元设置机器人位置
    bool robot_position_set_;
    // Number of times the cell is visited by the robot   这个单元机器人访问过多少次
    int visit_count_;
    // Indices of the viewpoints within this cell.    这个单元里面视点的ID
    std::vector<int> viewpoint_indices_;
    // Indices of other cells that are connected by a path.      通过一个路径与其他单元相连的单元ID
    std::vector<int> connected_cell_indices_;
    std::unordered_set<int> connected_cell_set_;  // O(1)查找
    // Indices of connected keypose graph nodes   关键位姿图上的 相连结点的ID  连通的  结点  索引
    std::vector<int> keypose_graph_node_indices_;
    //网格  新增属性    拼接图  在  这个网里面  存在的 节点  编号  用于 快速查找    merger_graph_  中相同的节点  做节点  融合
    std::vector<int> merger_graph_node_indices_;

    // Whether this cell is in the planning horizon, which consists of nine cells around the robot.   该单元是否在规划范围内
    bool in_horizon_;
    // ID of the keypose where viewpoints in this cell can be observed 关键位姿ID
    int keypose_id_;
    // Position of the highest score viewpoint   最高分数视点   的位置
    Eigen::Vector3d viewpoint_position_;
    // Position for connecting the cell to the global roadmap     此单元连接   全局路线图的位置
    Eigen::Vector3d roadmap_connection_point_;
    // Path to the nearest keypose on the keypose graph     在关键位姿图上  最靠近关键位姿的路径
    nav_msgs::msg::Path path_to_keypose_graph_;
    // If the path has been added to the keypose graph   是否这个路径加入关键位姿图
    bool path_added_to_keypose_graph_;
    // If the roadmap connection point has been added to the cell  是否连接路径图的点被加入这个单元
    bool roadmap_connection_point_set_;
    //新增一个积累当前单元连接其他单元的  容器
    std::vector<int> long_term_connected_cell_indices_; //长期的连接，  只有断开连接  没有清空操作
    int MTSP_graph_index_find_exploring_cell_; //  机器人在  图上的  索引

    //merger_graph添加 在 网格的节点  所不同 的  机器人 编号
    std::set<int> merger_graph_robot_id_set_;
    //merger_graph  多个机器人  连接 边 属性
    std::map<int, std::map<int, bool>> robotmin_robotmax_isconnected_;
};

//网格世界   类
class GridWorld
{
public:
    explicit GridWorld(rclcpp::Node::SharedPtr nh);
    explicit GridWorld(int row_num = 1, int col_num = 1, int level_num = 1, double cell_size = 6.0,
                     double cell_height = 6.0, int nearby_grid_num = 5);
    ~GridWorld() = default;
    
    void ReadParameters(rclcpp::Node::SharedPtr nh);
    void UpdateNeighborCells(const geometry_msgs::msg::Point& robot_position);   //更新机器人邻接的单元
    void UpdateRobotPosition(const geometry_msgs::msg::Point& robot_position);
    void UpdateCellKeyposeGraphNodes(const std::shared_ptr<keypose_graph_ns::KeyposeGraph> &keypose_graph);

    bool IndInBound(int ind)
    {
        return subspaces_->InRange(ind);
    }
    
    // Get the cell index where the robot is currently in.     得到当前机器人所在   cell 的ID
    bool AreNeighbors(int cell_ind1, int cell_ind2);   //1，2   是否邻接
    int GetCellInd(double qx, double qy, double qz);   //输入位姿获得 Cell   的ID
    void GetCellSub(int& row_idx, int& col_idx, int& level_idx, double qx, double qy, double qz);
    Eigen::Vector3i GetCellSub(const Eigen::Vector3d& point);
    
    // Get the visualization markers for Rviz display.  可视化
    void GetMarker(visualization_msgs::msg::Marker& marker);
    // Get the visualization pointcloud for debugging purpose
    void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud);

    bool Initialized()
    {
        return initialized_;
    }
    
    void SetUseKeyposeGraph(bool use_keypose_graph)
    {
        use_keypose_graph_ = use_keypose_graph;
    }
    
    void AddViewPointToCell(int cell_ind, int viewpoint_ind);  //给单元 添加视点
    void AddGraphNodeToCell(int cell_ind, int node_ind);  //添加图结点给单元
    void ClearCellViewPointIndices(int cell_ind);   //清理单元内的候选视点
    std::vector<int> GetCellViewPointIndices(int cell_ind);//得到单元的候选视点
    void GetNeighborCellIndices(const Eigen::Vector3i& center_cell_sub, const Eigen::Vector3i& neighbor_range,
                              std::vector<int>& neighbor_indices);   //得到给定单元的   邻接单元ID
    void GetNeighborCellIndices(const geometry_msgs::msg::Point& position, const Eigen::Vector3i& neighbor_range,
                              std::vector<int>& neighbor_indices);  //得到给定位置的   邻接单元ID
    void GetExploringCellIndices(std::vector<int>& exploring_cell_indices);  //得到探索单元的ID
    std::vector<int> GetCellConnectedCellIndices(int cell_ind);  //获取单元的连接单元ID列表
    CellStatus GetCellStatus(int cell_ind);  //得到单元状态
    CellStatus GetCellStatus_world(int cell_ind);  //得到单元状态
    void SetCellStatus(int cell_ind, CellStatus status);  //设置单元状态
    geometry_msgs::msg::Point GetCellPosition(int cell_ind); //得到给定单元ID的 位置
    void SetCellRobotPosition(int cell_ind, const geometry_msgs::msg::Point& robot_position);  //设置单元机器人位置
    geometry_msgs::msg::Point GetCellRobotPosition(int cell_ind);  //得到单元  机器人位置
    void CellAddVisitCount(int cell_ind);   //单元添加访问次数
    int GetCellVisitCount(int cell_ind); //得到单元的访问次数
    bool IsRobotPositionSet(int cell_ind); //单元是否是机器人位置
    void Reset();
    int GetCellStatusCount(grid_world_ns::CellStatus status);  //得到单元状态数量
    //修改状态改变
    void UpdateCellStatus_(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager);   //更新单元状态  通过视点管理类

    // 新增 修改  使用  merger  graph  做
    exploration_path_ns::ExplorationPath SolveGlobalMdvrp_merger_graph(
        const std::shared_ptr<viewpoint_manager_ns::ViewPointManager> &viewpoint_manager,
        std::vector<int> &ordered_cell_indices,
        bool &is_global_tsp,
        std::vector<rclcpp::Client<tare_planner::srv::RequestPath>::SharedPtr>& request_path_client_list_,
        std::shared_ptr<keypose_graph_ns::KeyposeGraph> &keypose_graph,
        std::shared_ptr<merger_graph_ns::MergerGraph> &merger_graph,
        std::map<int,geometry_msgs::msg::Point>&  other_robot_position_map,
        std::shared_ptr<skeleton_graph_ns::SkeletonGraph>& skeleton_graph);

    inline void SetCurKeyposeGraphNodeInd(int node_ind) //设置当前关键位姿图结点ID
    {
        cur_keypose_graph_node_ind_ = node_ind;
    }
    
    inline void SetCurKeyposeGraphNodePosition(geometry_msgs::msg::Point node_position)//设置当前关键位姿图结点位置
    {
        cur_keypose_graph_node_position_ = node_position;
    }
    
    inline void SetCurKeyposeID(int keypose_id)//设置当前位姿ID
    {
        cur_keypose_id_ = keypose_id;
    }

    inline void SetCurKeypose(const Eigen::Vector3d& cur_keypose) //设置当前位置
    {
        cur_keypose_ = cur_keypose;
    }
    
    int GetCellKeyposeID(int cell_ind);
    
    void SetHomePosition(const Eigen::Vector3d& home_position)
    {
        home_position_ = home_position;
        set_home_ = true;
    }
    
    bool HomeSet()
    {
        return set_home_;
    }
    
    bool IsReturningHome()
    {
        return return_home_;
    }
    
    void GetCellViewPointPositions(std::vector<Eigen::Vector3d>& viewpoint_positions);   //得到单元  视点位置
    void AddPathsInBetweenCells(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager> &viewpoint_manager, const std::shared_ptr<keypose_graph_ns::KeyposeGraph> &keypose_graph);
    bool PathValid(const nav_msgs::msg::Path& path, int from_cell_ind, int to_cell_ind);     //判定  路经是否存在于两个单元之间
    bool HasDirectKeyposeGraphConnection(const std::shared_ptr<keypose_graph_ns::KeyposeGraph> &keypose_graph, const Eigen::Vector3d &start_position, const Eigen::Vector3d &goal_position);

    //添加函数  打包网格世界邻接单元ID集  及状态  更新结点
    void GetUpdateNeighbor_NewGridWorldCellStatus_(std::vector<int>&  UpdateNeighbor_GridWorldCellStatus_code_);
    //添加函数  更新网格世界单元状态  从其他机器人上面
    void UpdateGridWorldCellFromOtherRobots(std::map<int,int>& Update_Grid_World_ID_and_Statu_);
    
    //添加函数  计算  融合图（连通图）中 两点 之间的最短距离
    double GetShortestPath_Fuse_Grapher(fuse_grapher& Fuse_Graoher_,int start_id, int target_id);
    
    //添加函数  判断局部路径规划   类型
    bool GetLocalPlanningType(const exploration_path_ns::ExplorationPath& global_path);
    
    //外部可访问
    exploration_path_ns::ExplorationPath   last_global_goal_path_;//融合图所做的目标，只要融合成功，且 融合目标还是探索状态   选用上一次的全局目标。
    
    //添加函数返回当前机器人状态
    grid_world_ns::RobotStatus GetRobotStatu();

    //添加函数   获取靠近局部规划框的  探索子网格到机器人的路径
    void Get_subgrid_paths(std::vector<exploration_path_ns::ExplorationPath>& near_localcoverage_subgrid_paths,
    std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);

    // 添加函数   merger_graph_  插入点  在这里维护
    void AddNode2MergerGraph(const std::shared_ptr<merger_graph_ns::MergerGraph>& merger_graph,
    std::unordered_map<int,tare_planner::msg::SharedMergerInfor>& Shared_Infor_map);
    
    //  添加  函数   更新  在邻接单元里面  添加 不同机器人之间的边
    void AddDiffRobotEdge2MergerGraphOnNeighborCell(const std::shared_ptr<merger_graph_ns::MergerGraph> &merger_graph,
                                                    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager> &viewpoint_manager,
                                                    std::vector<std::pair<int, int>>& add_edge);

    //新增分配策略  
    std::string allocation_strategy_;
    
    double GetkCellSize() const
    {
        return kCellSize;
    }

private:
    int kRowNum;
    int kColNum;
    int kLevelNum;
    double kCellSize;
    double kCellHeight;
    int KNearbyGridNum;   //邻接网格数量
    int kMinAddPointNumSmall;
    int kMinAddPointNumBig;
    int kMinAddFrontierPointNum;
    int kCellExploringToCoveredThr;    //单元探索 到 覆盖阈值
    int kCellCoveredToExploringThr;   //单元覆盖 到探索阈值
    int kCellExploringToAlmostCoveredThr;  //单元探索到全部 覆盖 阈值
    int kCellAlmostCoveredToExploringThr; //单元全部覆盖  到探索阈值
    int kCellUnknownToExploringThr; //单元未知到  探索阈值
    double kLocal_planning_radius;  //局部规划半径
    int num_local_neighbor_cell_reachable;
    int num_world_neighbor_cell_reachable;

    std::vector<Cell> cells_; //网格单元    
    std::shared_ptr<grid_ns::Grid<Cell>> subspaces_;  //     网格空间     就是使用的这个    只  共享覆盖信息
    std::shared_ptr<grid_ns::Grid<Cell>> subspaces_local_;    //新增            一个本地网格空间   保留  本地的  状态
    std::shared_ptr<grid_ns::Grid<Cell>> subspaces_world_;    //新增           共享覆盖和探索信息
    bool initialized_;
    bool use_keypose_graph_; //
    int cur_keypose_id_;  //当前关键位姿  ID
    int cur_robot_id_;//当前 id
    geometry_msgs::msg::Point robot_position_;  //机器人位置
    geometry_msgs::msg::Point origin_;   //起点
    std::vector<int> neighbor_cell_indices_;  //邻接单元ID
    std::vector<int> almost_covered_cell_indices_;   //大部分覆盖的单元ID
    std::vector<std::pair<int, int>> to_connect_cell_indices_;  //相连的单元ID
    std::vector<nav_msgs::msg::Path> to_connect_cell_paths_; //连接单元的路径
    Eigen::Vector3d home_position_; //起点位置
    Eigen::Vector3d cur_keypose_; 
    bool set_home_; //设置起点
    bool return_home_; //返回起点
    bool is_fuse_;
    geometry_msgs::msg::Point cur_keypose_graph_node_position_;  //当前 关键位姿图结点位姿
    int cur_keypose_graph_node_ind_;
    int cur_robot_cell_ind_;  //当前 机器人 单元ID 
    int prev_robot_cell_ind_; //之前机器人单元ID
    int last_robot_cell_ind_; //上一次机器人单元ID

    //新增机器人状态
    RobotStatus  robot_statu_;

    // 新增断点累计  
    int Request_breakpoint_count_;    //  断点累计，当一次  Request  断点累计 到   10次之后，说明当前点不可通行  目标点是误点
    //新增  far_planner   请求信息
    int Request_robot_id_; //请求路径的  机器人id       初始化0 
    // 请求路经的  中间点
    geometry_msgs::msg::Point Intermediate_node_; //中间节点坐标

    //新增全局规划目标
    // exploration_path_ns::ExplorationPath   last_global_goal_path_;//融合图所做的目标，只要融合成功，且 融合目标还是探索状态   选用上一次的全局目标。

    //新增  网格图
    MTSP_grid_graph MTSP_grid_graph_;
    geometry_msgs::msg::Point goal_position_;//  记录准确的  全局规划     far目标点   和  Goal_tsp  目标点
    int far_breakpoint_coint_;//坏点累计
};
}  // namespace grid_world_ns
