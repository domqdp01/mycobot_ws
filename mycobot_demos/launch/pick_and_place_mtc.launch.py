#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    package_name_moveit_config = 'mycobot_moveit_config'
    pkg_share_moveit_config_temp = FindPackageShare(package=package_name_moveit_config)

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='mycobot_280',
        description='Name of the robot to use')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation clock if true')

    def configure_setup(context):
        robot_name_str = LaunchConfiguration('robot_name').perform(context)
        use_sim_time = LaunchConfiguration('use_sim_time')

        pkg_share_moveit_config = pkg_share_moveit_config_temp.find(package_name_moveit_config)
        config_path = os.path.join(pkg_share_moveit_config, 'config', robot_name_str)

        joint_limits_file_path      = os.path.join(config_path, 'joint_limits.yaml')
        kinematics_file_path        = os.path.join(config_path, 'kinematics.yaml')
        moveit_controllers_file_path = os.path.join(config_path, 'moveit_controllers.yaml')
        srdf_model_path             = os.path.join(config_path, f'{robot_name_str}.srdf')
        pilz_cartesian_limits_file_path = os.path.join(config_path, 'pilz_cartesian_limits.yaml')

        moveit_config = (
            MoveItConfigsBuilder(robot_name_str, package_name=package_name_moveit_config)
            .trajectory_execution(file_path=moveit_controllers_file_path)
            .robot_description_semantic(file_path=srdf_model_path)
            .joint_limits(file_path=joint_limits_file_path)
            .robot_description_kinematics(file_path=kinematics_file_path)
            .planning_pipelines(
                pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
                default_planning_pipeline="ompl"
            )
            .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
            .to_moveit_configs()
        )

        pick_place_demo = Node(
            package="mycobot_demos",
            executable="mtc_node",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': use_sim_time},
            ],
        )

        return [pick_place_demo]

    ld = LaunchDescription()
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(OpaqueFunction(function=configure_setup))

    return ld