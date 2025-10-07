# 无人车编队

## 项目简介

本项目实现了一个基于 ROS2（Foxy 版本）的阿克曼无人车编队系统，涵盖了阿克曼无人单车仿真搭建、仿真环境搭建、单车定位导航、编队控制导航等内容。项目重点在于编队框架的搭建和流程跑通，单车定位导航算法采用 ROS2 官方算法，仅为保证编队稳定性做了部分修改，未改动算法核心。编队控制实现了一个基础框架，并基于此简单实现了线性序列编队，同时支持非线性编队，只需配置车间关系即可。

---

## 目录

1. [阿克曼单车仿真](#阿克曼单车仿真)
2. [仿真环境搭建](#仿真环境搭建)
3. [启动仿真小车和环境](#启动仿真小车和环境)
4. [建图](#建图)
5. [单车导航](#单车导航)
6. [编队控制（autofleet包）](#编队控制autofleet包)
7. [开发中遇到的问题及解决方案](#开发中遇到的问题及解决方案)

---

## 阿克曼单车仿真

> 实现在 `ackermann` 包的 `urdf` 文件夹中，使用xacro机器人描述文件：

- **a. acuator 文件夹（车轮）**
  - 车轮分为前轮和后轮，后轮为驱动轮，前轮为转向轮。
  - 后轮实现：`rear_wheel_link`（用于移动）和一个 joint（连接 `base_link` 和 `rear_wheel_link`）。
  - 前轮实现：`front_wheel_link` 和 `steer_wheel_link`，用于控制转向；两个 joint 分别连接 `base_link` 和 `steer_wheel_link`、`steer_wheel_link` 和 `front_wheel_link`。

- **b. sensor 文件夹（传感器）**
  - 描述传感器的安装位置。

- **c. base.urdf.xacro 文件**
  - 声明无人车底盘（`base_link`）、地面 link（`base_footprint`）、虚拟方向盘（`virtual_steer_link`），以及相关 joint。
  - `virtual_steer_link` 及其 joint 为兼容控制插件（`libgazebo_ros_joint_state_publisher.so`）而设。

- **d. plugins 文件夹**
  - 包含 `libgazebo_ros_joint_state_publisher.so`（发布选定关节状态）、`gazebo_ros_joint_pose_trajectory.so`（读取 JointTrajectory 消息并移动关节）、`libgazebo_ros_ackermann_drive.so`（Ackermann 驱动插件，发布 odom 并接收 cmd_vel 控制小车移动）。

- **e. common_inertial.xacro 文件**
  - 定义各种 link 的质量和惯性。

- **f. robot.urdf.xacro 文件**
  - 组织所有无人车组件，调用控制插件和传感器插件。
  - 针对编队做了特殊处理：所有无人车命名通过参数传递，并根据机器人名字给插件命名空间，实现多机器人隔离，每个机器人拥有独立 tf 子树。

---

## 仿真环境搭建

- `world` 文件夹，使用 Gazebo 画了几堵墙。
- 参考：[CSDN 博客](https://blog.csdn.net/A981012/article/details/105332233)

---

## 启动仿真小车和环境

- `display_robot.launch.py` 启动两个节点：
  - `spawn_entity.py`：在 Gazebo 中启动机器人实例（输入为 urdf）。
  - `robot_state_publisher`：接收 joint 状态并发布 tf 树。

---

## 建图

- 使用 Google 的 Cartographer。
- 启动 `display_robot.launch.py` 和 Gazebo 节点后，再启动 `cartographer.launch.py`，生成 `map` 文件夹中的 pgm 和 yaml 文件。
- 后续定位导航会用到该 map 文件夹。

---

## 单车导航

- 启动 `single_robot.launch.py` 文件。
  - 需输入 `gazebo_pose.yaml`，配置机器人名字和初始化位置。
  - 调用 `gazebo.launch.py`（输入参数为 `simple_world.world`）启动 Gazebo。
  - 启动 `display_robot.launch.py` 启动无人车模型。
  - 启动 `navigations.launch.py` 启动 navigation2 导航。
    - `navigations.launch.py` 启动 `bringup_launch.py`（参数文件如 `nav2_robot1_params.yaml`）和 rviz 节点。
    - `bringup_launch.py` 启动 `navigation_launch.py` 和 `localization_launch.py`，对 tf 和 tf_static 做了处理，统一挂在 map 下，实现多机器人隔离。
  - 定位使用 amcl，全局路径规划用 A*，局部路径规划用 RPP。

---

## 编队控制（autofleet包）

- 主要实现编队控制基本功能，采用 C++ 行为树控制流程。
- 只有一个节点 `autofleet_node`，行为树加载不同阶段的 so 文件作为节点，包括 `"FormTeamState"`, `"MoveState"`, `"BreakTeamState"`，分为编队形成、行进、解散三个状态，顺序组织。
- 该节点还负责初始化 ROS 相关 client、service、publisher、subscription、action、timer，以及编队信息（队形等）。

### 主要类说明

- **State 基类**：实现所有状态的公共方法和属性，如 ROS 节点指针、tf 变换、期望位姿计算、目标点判断等。
- **FormTeamState 类**：启动领头车，判断跟随车是否可启动，全部启动后头车停止，跟随车到达队列正确位置后结束状态。
- **MoveState 类**：启动领头车，周期计算跟随车期望位置并发布目标点，控制队形误差，头车到达目标点后结束状态。
- **BreakTeamState 类**：领头车停止后，计算跟随车停放位置，按序重设目标点并移动到指定位置。
- **laserScan_filter 类**：激光数据过滤，去除编队中相邻车辆的激光点，解决动态障碍物残留问题。
- **util 类**：丰富 ROS 日志，增加文件名、函数、行数、线程 id、不同等级颜色等信息，便于调试。

---

## 开发中遇到的问题及解决方案

### 1. 动态障碍物残留

- 原因：激光雷达分辨率有限，costmap 清除机制导致部分点无法及时清除，导航 abort。
- 解决：因为在costmap更新的时候是根据激光雷达的数据先mark然后clear，clear的方法是根据扫描到的点和原点连成一条线，线经过的costmap中对应的栅格会从mark被标记成free，基于这个原理，如果激光雷达分辨率不够，那么就会出现头一帧扫描到某个点，下一帧由于物体移动到了激光雷达的两条线之间扫描不到，就会导致这个点一直不会被clear掉，导致导航abort。一般的解决方案是对clear点云中的点进行领域拓展，即如果扫描到了某个点，那么将其上下左右四个领域内都生成一个点，用于clear，这样就变相放大了一个点的影响范围。但是在本文的实验环境中发现，由于使用的是单线激光雷达，且激光雷达位于车辆上方，因此某个车扫描到相邻车辆的点只有一个，不能使用上面说的方法，因此直接删除掉扫描的相邻车的点，这样做只能在编队中用，因为由于存在编队控制，不会使编队中发生车间碰撞。


### 2. 回调死锁问题

- 原因：ROS2 回调组默认互斥，定时器和 client 回调在同组时会死锁。
- 解决：ros2存在一个回调组的概念，如果回调函数处于同一个回调组（如果不指定回调组，ros2默认是这样的），且该回调组是互斥回调组，那么在处理某个回调函数的过程中，是不会触发同一回调组的其他回调函数的，在该项目中主要出现在使用定时器周期查询navigation2中的lifecycleManager的状态是否是actived，查询方法就是向lifecycleManager中的一个服务发送信息并等待一个bool类型的值，但是在第一次实现过程中发现async_send_goal总会超时，在之后查阅ros wiki中发现了这个问题的解决方案，就是将定时器和client的回调放在不同回调组中即可，也可以使用可重入回调组。

### 3. robot_state_publisher 和 spawn_entity 启动顺序问题

- 原因：两个节点无先后顺序，spawn_entity 可能收不到 urdf。
- 解决：在仿真期间，存在两个节点 robot_state_publisher 和 spawn_entity，前者是用来发布静态tf树和重发robot_description(机器人的urdf)，后者是根据机器人的urdf来构建gazebo中的一个仿真实例。但是在launch中启动这两个节点是没有先后顺序的，那么如何保证spawn_entity能接收到机器人的urdf呢，原因就是在两个节点对topic：robot_decription消息的配置中加了qos策略：持久性中的瞬态本地（QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL），这个策略可以使发布的消息存储在队列中等待被取走。

---

## 节点关系图和tf树
<img width="12587" height="1448" alt="autofleet_tf_tree" src="https://github.com/user-attachments/assets/c7605f42-01d6-4756-bbb9-dfa01ded3145" />

<img width="6459" height="7313" alt="autofleetrosgraph" src="https://github.com/user-attachments/assets/fbefbd47-c4aa-4c1a-83c1-6aca6111e19a" />

## 编队效果展示
![aufofleet](https://github.com/user-attachments/assets/fae45191-ca17-40e3-b4b9-581de63554fd)


## 参考

- [CSDN 博客：Gazebo 环境搭建](https://blog.csdn.net/A981012/article/details/105332233)

---

## 结语

本项目重点在于编队框架和流程的完整实现，单车算法采用 ROS2 官方方案。编队控制采用行为树，支持多种队形，便于扩展和维护。欢迎交流与反馈！
