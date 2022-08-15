cmake_minimum_required(VERSION 3.5)
project(mcr_uwb)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(protocol REQUIRED)

include_directories(
  include
)

set(executable_name mcr_uwb)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_util
  protocol
  geometry_msgs
  builtin_interfaces
  tf2_ros
)

add_library(${library_name} SHARED
  src/mcr_uwb.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${executable_name}
  ${dependencies}
)

# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
