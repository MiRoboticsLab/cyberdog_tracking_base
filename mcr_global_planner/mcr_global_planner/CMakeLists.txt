cmake_minimum_required(VERSION 3.5)
project(mcr_global_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -O0)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(mcr_nav_grid REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(ompl REQUIRED)


set(library_name "mcr_global_planner")
add_library(${library_name} SHARED
              src/cost_interpreter.cpp
              src/mcr_global_planner.cpp
              src/mcr_curve_planner.cpp
              src/mcr_spline_planner.cpp)

target_include_directories(${library_name} PUBLIC
  ${OMPL_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${library_name}
  "rclcpp"
  "geometry_msgs"
  "nav2_core"
  "nav2_util"
  "pluginlib"
  "tf2_ros"
  "nav2_costmap_2d"
  "nav_2d_msgs"
  "mcr_nav_grid"
  "nav_2d_utils"
  "ompl"
)
target_link_libraries(${library_name} ${OMPL_LIBRARIES})
  
add_executable(global_planner 
              src/global_planner.cpp)

target_include_directories(global_planner PUBLIC
  ${OMPL_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  global_planner
  "rclcpp"
  "geometry_msgs"
  "nav2_core"
  "nav2_util"
  "pluginlib"
  "tf2_ros"
  "nav2_costmap_2d"
  "nav_2d_msgs"
  "mcr_nav_grid"
  "nav_2d_utils"
  "ompl"
  )


# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core mcr_global_planner_plugin.xml)


install(TARGETS ${library_name} 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS global_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch 
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params
    DESTINATION share/${PROJECT_NAME})


install(DIRECTORY include/
      DESTINATION include)    

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_export_definitions("PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_package()
