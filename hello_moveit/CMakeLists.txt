cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_ros REQUIRED)

# include directories
include_directories(
  include
)

# executable
add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(pick_and_place_plan_around_objects src/pick_and_place_plan_around_objects.cpp)

target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)

target_include_directories(pick_and_place_plan_around_objects PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(pick_and_place_plan_around_objects PUBLIC c_std_99 cxx_std_17)

# link dependencies
ament_target_dependencies(
  hello_moveit
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  tf2_ros
)

ament_target_dependencies(
  pick_and_place_plan_around_objects
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  tf2_ros
)
# install
install(TARGETS hello_moveit pick_and_place_plan_around_objects
  DESTINATION lib/${PROJECT_NAME})

# testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()