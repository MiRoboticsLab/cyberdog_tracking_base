ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
ros2 service call /lifecycle_manager_simulator/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
# ros2 service call /is_target_insight_inused std_srvs/srv/SetBool "{data: true}" # side follow use this.
ros2 action send_goal /tracking_target mcr_msgs/action/TargetTracking "{relative_pos: 1}" --feedback
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 1}"
ros2 service call /lifecycle_manager_simulator/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 1}"
