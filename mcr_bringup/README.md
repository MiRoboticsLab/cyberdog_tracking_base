# mcr_bringup

```
.
├── behavior_trees
│   ├── auto_docking.xml
│   ├── automatic_recharge_old.xml
│   ├── automatic_recharge.xml
│   ├── navigate_to_pose_w_replanning_and_recovery.xml
│   └── target_tracking.xml
├── CMakeLists.txt
├── launch
│   ├── bringup_dock_launch.py
│   ├── bringup_follow_launch.py
│   ├── bringup_follow_only_launch.py
│   └── bringup_navigate_to_pose_launch.py
├── package.xml
├── params
│   ├── auto_charging.yaml
│   ├── auto_docking.yaml
│   ├── follow_params.yaml
│   ├── follow_params.yaml.bak
│   ├── nav2_params.yaml
│   ├── navigate_to_pose_params.yaml
│   └── recoveries_params.yaml
└── README.md
```

## behavior_trees

- auto_docking.xml 为自主回充的行为树
- navigate_to_pose_w_replanning_and_recovery.xml 位导航的行为树，继承自nav2_bt_navigator
- target_tracking.xml 位跟随功能的行为树
  
## params

- auto_docking.yaml 为自主回充配置
- follow_params.yaml 为跟随的配置
- navigate_to_pose_params.yaml 为导航配置
- recoveries_params.yaml 为恢复行为相关配置，为以上三个功能通用
