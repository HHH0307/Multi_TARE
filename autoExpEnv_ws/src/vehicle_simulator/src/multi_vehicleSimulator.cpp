// 包含必要的头文件
#include <math.h>                     // 数学函数库
#include <time.h>                     // 时间相关函数
#include <stdio.h>                    // 标准输入输出
#include <stdlib.h>                   // 标准库函数
#include <chrono>                     // C++时间库
#include <iostream>                   // 标准输入输出流
#include <string>
#include "rclcpp/rclcpp.hpp"          // ROS2 C++客户端库
#include "rclcpp/time.hpp"            // ROS2时间处理
#include "rclcpp/clock.hpp"           // ROS2时钟
#include "builtin_interfaces/msg/time.hpp" // ROS2内置时间消息

// ROS2消息类型
#include "nav_msgs/msg/odometry.hpp"  // 里程计消息
#include "sensor_msgs/msg/point_cloud2.hpp" // 点云消息
#include <sensor_msgs/msg/joy.hpp>    // 游戏手柄消息
#include <std_msgs/msg/float32.hpp>   // 浮点数消息
#include <std_msgs/msg/bool.hpp>      // 布尔值消息
#include <nav_msgs/msg/path.hpp>      // 路径消息
#include <geometry_msgs/msg/twist_stamped.hpp> // 带时间戳的速度消息
#include <geometry_msgs/msg/point_stamped.h> // 带时间戳的点消息
#include <geometry_msgs/msg/polygon_stamped.h> // 带时间戳的多边形消息
#include <sensor_msgs/msg/imu.h>      // IMU消息

// Gazebo相关消息
#include <gazebo_msgs/msg/model_state.hpp> // Gazebo模型状态
#include <gazebo_msgs/msg/entity_state.hpp> // Gazebo实体状态
#include <gazebo_msgs/srv/set_entity_state.hpp> // Gazebo设置实体状态服务

// TF2相关
#include "tf2/transform_datatypes.h"  // TF2变换数据类型
#include "tf2_ros/transform_broadcaster.h" // TF2变换广播器
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // TF2几何消息转换

// OpenCV
#include <opencv2/opencv.hpp>         // OpenCV核心功能
#include <opencv2/highgui/highgui.hpp> // OpenCV GUI

// PCL点云库
#include <pcl/filters/voxel_grid.h>   // 体素网格滤波器
#include <pcl/kdtree/kdtree_flann.h>  // KD树搜索
#include <pcl_conversions/pcl_conversions.h> // PCL与ROS消息转换
#include <pcl/point_cloud.h>          // 点云基础类
#include <pcl/point_types.h>          // 点类型定义

// ROS2消息过滤器
#include "message_filters/subscriber.h" // 消息订阅器
#include "message_filters/synchronizer.h" // 消息同步器
#include "message_filters/sync_policies/approximate_time.h" // 近似时间同步策略
#include "rmw/types.h"                // ROS中间件类型
#include "rmw/qos_profiles.h"         // ROS服务质量配置

using namespace std; // 使用标准命名空间

const double PI = 3.1415926; // 定义π值

// 参数变量定义
bool use_gazebo_time = false; // 是否使用Gazebo时间
double cameraOffsetZ = 0;     // 相机Z轴偏移
double sensorOffsetX = 0;     // 传感器X轴偏移
double sensorOffsetY = 0;     // 传感器Y轴偏移
double vehicleHeight = 0.75;  // 车辆高度
double terrainVoxelSize = 0.05; // 地形体素大小
double groundHeightThre = 0.1; // 地面高度阈值
bool adjustZ = false;         // 是否调整Z轴
double terrainRadiusZ = 0.5;  // Z轴地形半径
int minTerrainPointNumZ = 10; // Z轴最小地形点数
double smoothRateZ = 0.2;     // Z轴平滑率
bool adjustIncl = false;      // 是否调整倾斜
double terrainRadiusIncl = 1.5; // 倾斜地形半径
int minTerrainPointNumIncl = 500; // 倾斜最小地形点数
double smoothRateIncl = 0.2;  // 倾斜平滑率
double InclFittingThre = 0.2; // 倾斜拟合阈值
double maxIncl = 30.0;        // 最大倾斜角度

// 系统初始化相关
const int systemDelay = 5;    // 系统延迟
int systemInitCount = 0;      // 系统初始化计数器
bool systemInited = false;    // 系统是否初始化完成

/* hqy 修改 */
string robot_name = "robot_1";

// 点云指针定义
pcl::PointCloud<pcl::PointXYZI>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZI>()); // 扫描数据
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 地形点云
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudIncl(new pcl::PointCloud<pcl::PointXYZI>()); // 倾斜地形点云
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>()); // 下采样地形点云

std::vector<int> scanInd; // 扫描索引

rclcpp::Time odomTime; // 里程计时间

// 车辆状态变量
float vehicleX = 0;     // 车辆X坐标
float vehicleY = 0;     // 车辆Y坐标
float vehicleZ = 0;     // 车辆Z坐标
float vehicleRoll = 0;  // 车辆横滚角
float vehiclePitch = 0; // 车辆俯仰角
float vehicleYaw = 0;   // 车辆偏航角

float vehicleYawRate = 0; // 车辆偏航率
float vehicleSpeed = 0;   // 车辆速度

// 地形状态变量
float terrainZ = 0;      // 地形高度
float terrainRoll = 0;   // 地形横滚角
float terrainPitch = 0;  // 地形俯仰角

// 状态堆栈
const int stackNum = 400; // 堆栈大小
float vehicleXStack[stackNum]; // X坐标堆栈
float vehicleYStack[stackNum]; // Y坐标堆栈
float vehicleZStack[stackNum]; // Z坐标堆栈
float vehicleRollStack[stackNum]; // 横滚角堆栈
float vehiclePitchStack[stackNum]; // 俯仰角堆栈
float vehicleYawStack[stackNum]; // 偏航角堆栈
float terrainRollStack[stackNum]; // 地形横滚角堆栈
float terrainPitchStack[stackNum]; // 地形俯仰角堆栈
double odomTimeStack[stackNum]; // 里程计时间堆栈
int odomSendIDPointer = -1; // 发送指针
int odomRecIDPointer = 0;   // 接收指针

pcl::VoxelGrid<pcl::PointXYZI> terrainDwzFilter; // 地形下采样滤波器

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubScanPointer; // 点云发布指针

// 扫描数据处理函数
void scanHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanIn)
{
  // 检查系统是否初始化完成
  if (!systemInited) {
    systemInitCount++;
    if (systemInitCount > systemDelay) {
      systemInited = true;
    }
    return;
  }

  // 获取扫描时间
  double scanTime = rclcpp::Time(scanIn->header.stamp).seconds();
  if (odomSendIDPointer < 0)
  {
    return;
  }
  
  // 同步时间
  while (odomTimeStack[(odomRecIDPointer + 1) % stackNum] < scanTime &&
         odomRecIDPointer != (odomSendIDPointer + 1) % stackNum)
  {
    odomRecIDPointer = (odomRecIDPointer + 1) % stackNum;
  }

  // 获取记录的状态
  double odomRecTime = odomTime.seconds();
  float vehicleRecX = vehicleX;
  float vehicleRecY = vehicleY;
  float vehicleRecZ = vehicleZ;
  float terrainRecRoll = terrainRoll;
  float terrainRecPitch = terrainPitch;

  // 如果使用Gazebo时间，从堆栈中获取状态
  if (use_gazebo_time)
  {
    odomRecTime = odomTimeStack[odomRecIDPointer];
    vehicleRecX = vehicleXStack[odomRecIDPointer];
    vehicleRecY = vehicleYStack[odomRecIDPointer];
    vehicleRecZ = vehicleZStack[odomRecIDPointer];
    terrainRecRoll = terrainRollStack[odomRecIDPointer];
    terrainRecPitch = terrainPitchStack[odomRecIDPointer];
  }

  // 计算三角函数值
  float sinTerrainRecRoll = sin(terrainRecRoll);
  float cosTerrainRecRoll = cos(terrainRecRoll);
  float sinTerrainRecPitch = sin(terrainRecPitch);
  float cosTerrainRecPitch = cos(terrainRecPitch);

  // 清空并转换点云数据
  scanData->clear();
  pcl::fromROSMsg(*scanIn, *scanData);
  pcl::removeNaNFromPointCloud(*scanData, *scanData, scanInd);

  // 点云坐标变换
  int scanDataSize = scanData->points.size();
  for (int i = 0; i < scanDataSize; i++)
  {
    // 第一步旋转：绕X轴（横滚角）
    float pointX1 = scanData->points[i].x;
    float pointY1 = scanData->points[i].y * cosTerrainRecRoll - scanData->points[i].z * sinTerrainRecRoll;
    float pointZ1 = scanData->points[i].y * sinTerrainRecRoll + scanData->points[i].z * cosTerrainRecRoll;

    // 第二步旋转：绕Y轴（俯仰角）
    float pointX2 = pointX1 * cosTerrainRecPitch + pointZ1 * sinTerrainRecPitch;
    float pointY2 = pointY1;
    float pointZ2 = -pointX1 * sinTerrainRecPitch + pointZ1 * cosTerrainRecPitch;

    // 平移变换
    float pointX3 = pointX2 + vehicleRecX;
    float pointY3 = pointY2 + vehicleRecY;
    float pointZ3 = pointZ2 + vehicleRecZ;

    // 更新点云坐标
    scanData->points[i].x = pointX3;
    scanData->points[i].y = pointY3;
    scanData->points[i].z = pointZ3;
  }

  // 发布5Hz的注册扫描消息
  sensor_msgs::msg::PointCloud2 scanData2;
  pcl::toROSMsg(*scanData, scanData2);
  scanData2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomRecTime * 1e9));
  scanData2.header.frame_id = robot_name+"/map"; /* hqy 修改 */
  pubScanPointer->publish(scanData2);
}

// 地形点云处理函数
void terrainCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloud2)
{
  // 如果不需要调整高度和倾斜，直接返回
  if (!adjustZ && !adjustIncl)
  {
    return;
  }

  // 清空并转换地形点云
  terrainCloud->clear();
  pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

  pcl::PointXYZI point;
  terrainCloudIncl->clear();
  int terrainCloudSize = terrainCloud->points.size();
  double elevMean = 0;
  int elevCount = 0;
  bool terrainValid = true;
  
  // 处理地形点云
  for (int i = 0; i < terrainCloudSize; i++)
  {
    point = terrainCloud->points[i];

    // 计算点到车辆的距离
    float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));

    // 在Z轴调整半径内的点
    if (dis < terrainRadiusZ)
    {
      if (point.intensity < groundHeightThre)
      {
        elevMean += point.z;
        elevCount++;
      }
      else
      {
        terrainValid = false;
      }
    }

    // 在倾斜调整半径内的点
    if (dis < terrainRadiusIncl && point.intensity < groundHeightThre)
    {
      terrainCloudIncl->push_back(point);
    }
  }

  // 计算平均高度
  if (elevCount >= minTerrainPointNumZ)
    elevMean /= elevCount;
  else
    terrainValid = false;

  // 调整Z轴高度
  if (terrainValid && adjustZ)
  {
    terrainZ = (1.0 - smoothRateZ) * terrainZ + smoothRateZ * elevMean;
  }

  // 下采样地形点云
  terrainCloudDwz->clear();
  terrainDwzFilter.setInputCloud(terrainCloudIncl);
  terrainDwzFilter.filter(*terrainCloudDwz);
  int terrainCloudDwzSize = terrainCloudDwz->points.size();

  // 检查点云数量是否足够
  if (terrainCloudDwzSize < minTerrainPointNumIncl || !terrainValid)
  {
    return;
  }

  // 使用OpenCV进行倾斜拟合
  cv::Mat matA(terrainCloudDwzSize, 2, CV_32F, cv::Scalar::all(0));
  cv::Mat matAt(2, terrainCloudDwzSize, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtA(2, 2, CV_32F, cv::Scalar::all(0));
  cv::Mat matB(terrainCloudDwzSize, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtB(2, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matX(2, 1, CV_32F, cv::Scalar::all(0));

  // 初始倾斜角度
  int inlierNum = 0;
  matX.at<float>(0, 0) = terrainPitch;
  matX.at<float>(1, 0) = terrainRoll;
  
  // 迭代拟合
  for (int iterCount = 0; iterCount < 5; iterCount++)
  {
    int outlierCount = 0;
    for (int i = 0; i < terrainCloudDwzSize; i++)
    {
      point = terrainCloudDwz->points[i];

      // 构建矩阵
      matA.at<float>(i, 0) = -point.x + vehicleX;
      matA.at<float>(i, 1) = point.y - vehicleY;
      matB.at<float>(i, 0) = point.z - elevMean;

      // 剔除异常点
      if (fabs(matA.at<float>(i, 0) * matX.at<float>(0, 0) + matA.at<float>(i, 1) * matX.at<float>(1, 0) -
               matB.at<float>(i, 0)) > InclFittingThre &&
          iterCount > 0)
      {
        matA.at<float>(i, 0) = 0;
        matA.at<float>(i, 1) = 0;
        matB.at<float>(i, 0) = 0;
        outlierCount++;
      }
    }

    // 求解线性方程组
    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    // 检查收敛
    if (inlierNum == terrainCloudDwzSize - outlierCount)
      break;
    inlierNum = terrainCloudDwzSize - outlierCount;
  }

  // 检查拟合结果是否有效
  if (inlierNum < minTerrainPointNumIncl || fabs(matX.at<float>(0, 0)) > maxIncl * PI / 180.0 ||
      fabs(matX.at<float>(1, 0)) > maxIncl * PI / 180.0)
  {
    terrainValid = false;
  }

  // 调整倾斜角度
  if (terrainValid && adjustIncl)
  {
    terrainPitch = (1.0 - smoothRateIncl) * terrainPitch + smoothRateIncl * matX.at<float>(0, 0);
    terrainRoll = (1.0 - smoothRateIncl) * terrainRoll + smoothRateIncl * matX.at<float>(1, 0);
  }
}

// 速度处理函数
void speedHandler(const geometry_msgs::msg::TwistStamped::ConstSharedPtr speedIn)
{
  vehicleSpeed = speedIn->twist.linear.x; // 获取线速度
  vehicleYawRate = speedIn->twist.angular.z; // 获取角速度
}

// 主函数
int main(int argc, char** argv)
{
  // 初始化ROS2节点
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("vehicleSimulator");

  // 声明参数
  nh->declare_parameter<bool>("use_gazebo_time", use_gazebo_time);
  nh->declare_parameter<double>("cameraOffsetZ", cameraOffsetZ);
  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<double>("vehicleHeight", vehicleHeight);
  nh->declare_parameter<double>("vehicleX", vehicleX);
  nh->declare_parameter<double>("vehicleY", vehicleY);
  nh->declare_parameter<double>("vehicleZ", vehicleZ);
  nh->declare_parameter<double>("terrainZ", terrainZ);
  nh->declare_parameter<double>("vehicleYaw", vehicleYaw);
  nh->declare_parameter<double>("terrainVoxelSize", terrainVoxelSize);
  nh->declare_parameter<double>("groundHeightThre", groundHeightThre);
  nh->declare_parameter<bool>("adjustZ", adjustZ);
  nh->declare_parameter<double>("terrainRadiusZ", terrainRadiusZ);
  nh->declare_parameter<int>("minTerrainPointNumZ", minTerrainPointNumZ);
  nh->declare_parameter<bool>("adjustIncl", adjustIncl);
  nh->declare_parameter<double>("terrainRadiusIncl", terrainRadiusIncl);
  nh->declare_parameter<int>("minTerrainPointNumIncl", minTerrainPointNumIncl);
  nh->declare_parameter<double>("InclFittingThre", InclFittingThre);
  nh->declare_parameter<double>("maxIncl", maxIncl);
  nh->declare_parameter<string>("robot_name", robot_name);  /* hqy 修改 */

  // 获取参数值
  nh->get_parameter("use_gazebo_time", use_gazebo_time);
  nh->get_parameter("cameraOffsetZ", cameraOffsetZ);
  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("vehicleHeight", vehicleHeight);
  nh->get_parameter("vehicleX", vehicleX);
  nh->get_parameter("vehicleY", vehicleY);
  nh->get_parameter("vehicleZ", vehicleZ);
  nh->get_parameter("terrainZ", terrainZ);
  nh->get_parameter("vehicleYaw", vehicleYaw);
  nh->get_parameter("terrainVoxelSize", terrainVoxelSize);
  nh->get_parameter("groundHeightThre", groundHeightThre);
  nh->get_parameter("adjustZ", adjustZ);
  nh->get_parameter("terrainRadiusZ", terrainRadiusZ);
  nh->get_parameter("minTerrainPointNumZ", minTerrainPointNumZ);
  nh->get_parameter("adjustIncl", adjustIncl);
  nh->get_parameter("terrainRadiusIncl", terrainRadiusIncl);
  nh->get_parameter("minTerrainPointNumIncl", minTerrainPointNumIncl);
  nh->get_parameter("InclFittingThre", InclFittingThre);
  nh->get_parameter("maxIncl", maxIncl);
  nh->get_parameter("robot_name", robot_name); /* hqy 修改 */

  // 创建订阅器
  auto subScan = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/"+robot_name+"/velodyne_points", 2, scanHandler); /* hqy 修改 */
  auto subTerrainCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/"+robot_name+"/terrain_map", 2, terrainCloudHandler); /* hqy 修改 */
  auto subSpeed = nh->create_subscription<geometry_msgs::msg::TwistStamped>("/"+robot_name+"/cmd_vel", 5, speedHandler); /* hqy 修改 */

  // 创建发布器
  auto pubVehicleOdom = nh->create_publisher<nav_msgs::msg::Odometry>("/"+robot_name+"/state_estimation", 5); /* hqy 修改 */
  nav_msgs::msg::Odometry odomData;
  odomData.header.frame_id = robot_name+"/map"; /* hqy 修改 */
  odomData.child_frame_id = robot_name+"/sensor"; /* hqy 修改 */
  // 创建TF广播器
  auto tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*nh);
  tf2::Stamped<tf2::Transform> odomTrans;
  geometry_msgs::msg::TransformStamped transformTfGeom ; 
  odomTrans.frame_id_ = robot_name+"/map"; /* hqy 修改 */

  // 创建Gazebo状态消息duiyu
  gazebo_msgs::msg::EntityState cameraState;
  cameraState.name = robot_name+"_camera"; /* hqy 修改 */
  gazebo_msgs::msg::EntityState lidarState;
  lidarState.name = robot_name+"_lidar"; /* hqy 修改 */
  gazebo_msgs::msg::EntityState robotState;
  robotState.name = robot_name+"_robot"; /* hqy 修改 */

  // 创建Gazebo服务客户端
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client = nh->create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");
  auto request  = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();

  // 创建点云发布器
  pubScanPointer = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/"+robot_name+"/registered_scan", 2); /* hqy 修改 */

  // 设置下采样滤波器参数
  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

  // 打印启动信息
  RCLCPP_INFO(nh->get_logger(), "Simulation started.");
  
  // 设置循环频率
  rclcpp::Rate rate(200);
  bool status = rclcpp::ok();
  while (status)
  {
    // 处理回调函数
    rclcpp::spin_some(nh);
    
    // 记录当前状态
    float vehicleRecRoll = vehicleRoll;
    float vehicleRecPitch = vehiclePitch;
    float vehicleRecZ = vehicleZ;

    // 更新车辆姿态
    vehicleRoll = terrainRoll * cos(vehicleYaw) + terrainPitch * sin(vehicleYaw);
    vehiclePitch = -terrainRoll * sin(vehicleYaw) + terrainPitch * cos(vehicleYaw);
    vehicleYaw += 0.005 * vehicleYawRate;
    if (vehicleYaw > PI)
      vehicleYaw -= 2 * PI;
    else if (vehicleYaw < -PI)
      vehicleYaw += 2 * PI;

    // 更新车辆位置
    vehicleX += 0.005 * cos(vehicleYaw) * vehicleSpeed +
                0.005 * vehicleYawRate * (-sin(vehicleYaw) * sensorOffsetX - cos(vehicleYaw) * sensorOffsetY);
    vehicleY += 0.005 * sin(vehicleYaw) * vehicleSpeed +
                0.005 * vehicleYawRate * (cos(vehicleYaw) * sensorOffsetX - sin(vehicleYaw) * sensorOffsetY);
    vehicleZ = terrainZ + vehicleHeight;

    // 获取当前时间
    odomTime = nh->now();
    
    // 更新状态堆栈
    odomSendIDPointer = (odomSendIDPointer + 1) % stackNum;
    odomTimeStack[odomSendIDPointer] = odomTime.seconds();
    vehicleXStack[odomSendIDPointer] = vehicleX;
    vehicleYStack[odomSendIDPointer] = vehicleY;
    vehicleZStack[odomSendIDPointer] = vehicleZ;
    vehicleRollStack[odomSendIDPointer] = vehicleRoll;
    vehiclePitchStack[odomSendIDPointer] = vehiclePitch;
    vehicleYawStack[odomSendIDPointer] = vehicleYaw;
    terrainRollStack[odomSendIDPointer] = terrainRoll;
    terrainPitchStack[odomSendIDPointer] = terrainPitch;

    // 发布200Hz的里程计消息
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(vehicleRoll, vehiclePitch, vehicleYaw);
    geometry_msgs::msg::Quaternion geoQuat;
    tf2::convert(quat_tf, geoQuat);

    odomData.header.stamp = odomTime;
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = vehicleX;
    odomData.pose.pose.position.y = vehicleY;
    odomData.pose.pose.position.z = vehicleZ;
    odomData.twist.twist.angular.x = 200.0 * (vehicleRoll - vehicleRecRoll);
    odomData.twist.twist.angular.y = 200.0 * (vehiclePitch - vehicleRecPitch);
    odomData.twist.twist.angular.z = vehicleYawRate;
    odomData.twist.twist.linear.x = vehicleSpeed;
    odomData.twist.twist.linear.z = 200.0 * (vehicleZ - vehicleRecZ);
    pubVehicleOdom->publish(odomData);

    // 发布200Hz的TF消息
    odomTrans.setRotation(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    odomTrans.setOrigin(tf2::Vector3(vehicleX, vehicleY, vehicleZ));
    transformTfGeom = tf2::toMsg(odomTrans);
    transformTfGeom.child_frame_id = robot_name+"/sensor";
    transformTfGeom.header.stamp = odomTime;
    tfBroadcaster->sendTransform(transformTfGeom);

    // 发布200Hz的Gazebo模型状态消息（用于Gazebo仿真）
    cameraState.pose.orientation = geoQuat;
    cameraState.pose.position.x = vehicleX;
    cameraState.pose.position.y = vehicleY;
    cameraState.pose.position.z = vehicleZ + cameraOffsetZ;
    request->state = cameraState;
    auto response = client->async_send_request(request);

    robotState.pose.orientation = geoQuat;
    robotState.pose.position.x = vehicleX;
    robotState.pose.position.y = vehicleY;
    robotState.pose.position.z = vehicleZ;
    request->state = robotState;
    response = client->async_send_request(request);

    quat_tf.setRPY(terrainRoll, terrainPitch, 0);
    tf2::convert(quat_tf, geoQuat);
    lidarState.pose.orientation = geoQuat;
    lidarState.pose.position.x = vehicleX;
    lidarState.pose.position.y = vehicleY;
    lidarState.pose.position.z = vehicleZ;
    request->state = lidarState;
    response = client->async_send_request(request);

    // 检查ROS状态
    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}