cmake_minimum_required(VERSION 3.5)
project(mcr_planner_plugins)

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

## System dependencies are found with CMake's conventions
#find_package(Eigen3 REQUIRED)
# Solution to find Eigen3 with Saucy.
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(mcr_global_planner REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(mcr_nav_grid REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_util REQUIRED)

add_library(mcr_planner_plugins src/astar.cpp src/dijkstra.cpp src/gradient_path.cpp
            src/grid_path.cpp src/von_neumann_path.cpp src/b_spliner.cpp src/bezier_spliner.cpp
            src/polynomial_interpolation.cpp src/polynomial_fitting.cpp)

target_include_directories(mcr_planner_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  mcr_planner_plugins
  "mcr_global_planner"
  "nav2_core"
  "mcr_nav_grid"
  "pluginlib"
  "nav2_util"
  "nav_2d_msgs"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(mcr_planner_plugins PRIVATE "MCR_PLANNER_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(mcr_planner_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(mcr_global_planner mcr_planner_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS mcr_planner_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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
ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR})
ament_export_libraries(mcr_planner_plugins)
ament_export_targets(export_${PROJECT_NAME})
ament_export_definitions("PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_package()
