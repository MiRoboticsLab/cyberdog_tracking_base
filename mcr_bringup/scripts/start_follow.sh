ros2 service call ${1}/lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
ros2 service call ${1}/lifecycle_manager_mcr_uwb/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
# ros2 service call ${1}/is_target_insight_inused std_srvs/srv/SetBool "{data: true}" # side follow use this.
ros2 action send_goal ${1}/tracking_target mcr_msgs/action/TargetTracking "{relative_pos: 1}" --feedback
ros2 service call ${1}/lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 1}"
ros2 service call ${1}/lifecycle_manager_mcr_uwb/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 1}"
