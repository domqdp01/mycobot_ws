cmake_minimum_required(VERSION 3.8)
project(mycobot_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_visual_tools REQUIRED)



# Copy necessary files to designated locations in the project
install (
  DIRECTORY  src
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS
  src/pick_and_place_actionclient.py
  DESTINATION lib/${PROJECT_NAME}
)
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
  control_msgs
  rclcpp_action
)
# install
install(TARGETS hello_moveit pick_and_place_plan_around_objects
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
