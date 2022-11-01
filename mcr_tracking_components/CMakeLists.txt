cmake_minimum_required(VERSION 3.5)
project(mcr_tracking_components CXX)

add_compile_options( -Wall -Wextra -Wpedantic -fpermissive -Wunused-parameter)
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(mcr_msgs REQUIRED)
find_package(dwb_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_recoveries REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(backward_ros REQUIRED)

nav2_package()

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav2_msgs
  mcr_msgs
  nav2_core
  dwb_core
  nav2_recoveries
  nav_msgs
  behaviortree_cpp_v3
  nav2_behavior_tree
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  nav2_util
)
add_library(nav2_compute_path_spline_poses_action_bt_node SHARED src/behavior_tree_nodes/compute_path_spline_poses_action.cpp)
list(APPEND plugin_libs nav2_compute_path_spline_poses_action_bt_node)

add_library(nav2_spin_and_search_action_bt_node SHARED src/behavior_tree_nodes/spin_and_search_action.cpp)
list(APPEND plugin_libs nav2_spin_and_search_action_bt_node)

add_library(nav2_tracking_mode_decider_node_bt_node 
            SHARED 
            src/behavior_tree_nodes/tracking_mode_decider_node.cpp
            src/behavior_tree_nodes/orientation_derivers.cpp
            )
list(APPEND plugin_libs nav2_tracking_mode_decider_node_bt_node)

add_library(nav2_target_updater_node_bt_node SHARED src/behavior_tree_nodes/target_updater_node.cpp)
list(APPEND plugin_libs nav2_target_updater_node_bt_node)

add_library(nav2_charger_updater_node_bt_node SHARED src/behavior_tree_nodes/charger_updater_node.cpp)
list(APPEND plugin_libs nav2_charger_updater_node_bt_node)

add_library(nav2_change_gait_node_bt_node SHARED src/behavior_tree_nodes/change_gait_node.cpp)
list(APPEND plugin_libs nav2_change_gait_node_bt_node)

add_library(nav2_seat_adjust_client_node_bt_node SHARED src/behavior_tree_nodes/seat_adjust_client_node.cpp)
list(APPEND plugin_libs nav2_seat_adjust_client_node_bt_node)

add_library(nav2_is_battery_in_charge_condition_bt_node SHARED src/behavior_tree_nodes/is_battery_in_charge_condition.cpp)
list(APPEND plugin_libs nav2_is_battery_in_charge_condition_bt_node)

add_library(nav2_exception_verify_bt_node SHARED src/behavior_tree_nodes/exception_verify_node.cpp)
list(APPEND plugin_libs nav2_exception_verify_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


add_library(fake_progress_checker SHARED src/controller_plugins/fake_progress_checker.cpp)
ament_target_dependencies(fake_progress_checker ${dependencies})
# prevent pluginlib from using boost
target_compile_definitions(fake_progress_checker PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_library(xy_goal_checker SHARED src/controller_plugins/xy_goal_checker.cpp)
ament_target_dependencies(xy_goal_checker ${dependencies})
# prevent pluginlib from using boost
target_compile_definitions(xy_goal_checker PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_library(unreachable_goal_checker SHARED src/controller_plugins/unreachable_goal_checker.cpp)
ament_target_dependencies(unreachable_goal_checker ${dependencies})
# prevent pluginlib from using boost
target_compile_definitions(unreachable_goal_checker PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_library(keep_target_insight_critic SHARED src/dwb_critics/keep_target_insight.cpp)
ament_target_dependencies(keep_target_insight_critic ${dependencies})
# prevent pluginlib from using boost
target_compile_definitions(keep_target_insight_critic PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_library(nav2_spin_and_search_recovery SHARED src/recoveries/spin_and_search.cpp)
ament_target_dependencies(nav2_spin_and_search_recovery  ${dependencies})
# prevent pluginlib from using boost
target_compile_definitions(nav2_spin_and_search_recovery PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_library(mcr_costmap_layers SHARED src/costmap_layers/obstacle_layer.cpp)
ament_target_dependencies(mcr_costmap_layers  ${dependencies})
# prevent pluginlib from using boost
target_compile_definitions(mcr_costmap_layers PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS
  ${plugin_libs} 
  fake_progress_checker 
  xy_goal_checker 
  unreachable_goal_checker 
  keep_target_insight_critic 
  nav2_spin_and_search_recovery
  mcr_costmap_layers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES behavior_tree_nodes.xml DESTINATION share/${PROJECT_NAME})
install(FILES controller_plugins.xml DESTINATION share/${PROJECT_NAME})
install(FILES orientation_derivers.xml DESTINATION share/${PROJECT_NAME})
install(FILES dwb_critics.xml DESTINATION share/${PROJECT_NAME})
install(FILES recoveries.xml DESTINATION share/${PROJECT_NAME})
install(FILES costmap_plugins.xml DESTINATION share/${PROJECT_NAME})
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   find_package(ament_cmake_gtest REQUIRED)
#   add_subdirectory(test)
# endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${plugin_libs} 
  fake_progress_checker
  xy_goal_checker 
  unreachable_goal_checker 
  keep_target_insight_critic 
  nav2_spin_and_search_recovery
  mcr_costmap_layers
)

ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_core controller_plugins.xml)
pluginlib_export_plugin_description_file(dwb_core dwb_critics.xml)
pluginlib_export_plugin_description_file(mcr_tracking_components orientation_derivers.xml)
pluginlib_export_plugin_description_file(nav2_core recoveries.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
ament_package()
