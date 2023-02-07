cmake_minimum_required(VERSION 3.5)
project(bt_navigators)
add_compile_options(-g -O0)
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(mcr_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(protocol REQUIRED)
find_package(cyberdog_debug REQUIRED)
find_package(cyberdog_common REQUIRED)
nav2_package()

include_directories(
  include
)

add_executable(bt_navigator_tracking
  src/bt_navigator_tracking.cpp
)

add_executable(bt_navigator_docking
  src/bt_navigator_docking.cpp
)

set(library_name bt_navigators_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  nav2_behavior_tree
  nav_msgs
  nav2_msgs
  mcr_msgs
  behaviortree_cpp_v3
  std_srvs
  nav2_util
  nav2_core
  tf2_ros
  protocol
  cyberdog_debug
  cyberdog_common
    
)

add_library(${library_name} SHARED
  src/navigators/target_tracking.cpp
  src/navigators/auto_docking.cpp
)

ament_target_dependencies(bt_navigator_tracking
  ${dependencies}
)

target_link_libraries(bt_navigator_tracking ${library_name})

ament_target_dependencies(bt_navigator_docking
  ${dependencies}
)

target_link_libraries(bt_navigator_docking ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS bt_navigator_tracking 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS bt_navigator_docking
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
