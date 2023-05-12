# mcr_tracking_components

主要是基于navigation2中的插件接口进行扩充，主要包括了如下几个种类的插件

## 行为树节点

### change_gait_node

用于调整狗子步态的节点

### is_battery_in_charge_condition

检测是否已经开始进行充电的节点

### charger_updater_node

获取自主回充时，充电桩的位置信息

### orientation_derivers

基于卡尔曼，用于推导目标运行方向的计算

### compute_path_spline_poses_action

修改自navigation2， 用于跟随

### seat_adjust_client_node

自主回充时，最后趴下之前，调整姿态的接口

### compute_path_to_pose_action

修改自navigation2，增加了一些跟随功能专用接口

### spin_and_search_action

当目标丢失时，通过自旋转来找回目标

### exception_verify_node

异常校验，用于系统发生不同的异常时匹配不同的恢复行为

### target_updater_node

目标管理节点，用于目标位置接收和处理

### follow_path_action

修改自navigation2的follow path节点，增加了一些跟随功能专用接口

### tracking_mode_decider_node

预留，用于自主决策跟随时狗子相对与目标的位置。

## 控制器插件

### fake_progress_checker

不管机器人是否存在运动，都认为狗子运行正常

### x_goal_checker

只检测x方向是否抵达

### unreachable_goal_checker

不管机器人是否抵达目标，都返回未抵达

### xy_goal_checker

只检测直线距离是否满足阈值，不考虑航向角

## costmap的layer插件

### obstacle_layer

修改自navigation2，订阅目标信息然后在传感器数据中扣除目标。


## dwb规划器critic插件

### keep_target_insight

为了保持目标能够一直保持在摄像头视野，利用目标数据所增加的dwb评判器

### keep_person_insight

为了保持目标能够一直保持在摄像头视野，利用目标上游数据（reid结果）所增加的dwb评判器

## 恢复行为的插件

### spin_and_search

当目标丢失时候，会一边旋转一边找回
