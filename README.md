# 本地部署
## 1 clone 本仓库代码
```shell
git clone https://github.com/HHH0307/Multi_TARE.git
```
## 2 进入 Multi_TARE 文件夹，选择分支，进行编译
```shell
cd Multi_TARE
```
```shell
git checkout humble
```
### 1.1 编译 autoExpEnv_ws 模块
```shell
cd autoExpEnv_ws
```
```shell
bash colcon_build_shell.sh
```
### 1.2 编译 multi_Tare_planner 模块
```shell
cd ..
```
```shell
cd multi_Tare_planner
```
```shell
bash colcon_build_tare.sh
```
## 3 运行仿真程序
以 **indoor 室内环境 - 3车** 为例
```shell
cd ..
```
```shell
cd start_shell
```
```shell
bash start_multi_indoor_3.sh
```
