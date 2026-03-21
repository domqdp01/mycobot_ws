#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class PickAndPlace(Node):


    def __init__(self):

        super().__init__('pick_and_place_action_client')

        # Set up arm trajectory action client for arm movement control
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        # Set up gripper action client for gripper control
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_action_controller/gripper_cmd'
        )

        # Wait for both action servers to be available
        self.get_logger().info('Waiting for action servers...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('Action servers connected!')

        # List of joint names for the robot arm
        self.joint_names = [
            'link1_to_link2', 'link2_to_link3', 'link3_to_link4', 'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange'
        ]

        # Define target and home positions for the arm
        self.red_cube_pose = [-0.674, -1.531, -0.119, 0.081, -0.283, 0.874]
        self.red_bin_pose = [0.636, -1.468, -0.195, 1.127, -0.283, 0.874]
        self.middle_pose = [-0.775, -0.497, -0.018, -0.006, -0.05, -0.874]
        self.home_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.blue_cube_pose = [-0.230, -0.699, -1.772, 0.850, 0.006, 1.324]
        self.middle_blue_pose = [-1.506, -0.931, -0.468, 0.0, 0.0, 0.0]
        self.blue_bin_pose = [-0.157, -1.367, -0.006, 0.485, 0.0, 0.0]


        # Create timer that triggers the control loop quickly after start (0.1 seconds)
        # self.create_timer(0.1, self.control_loop_callback)
        self.timer = self.create_timer(0.1, self.control_loop_callback)

    def send_arm_command(self, positions: list) -> None:
        """
        Send a command to move the robot arm to specified joint positions.

        Args:
            positions (list): List of 6 joint angles in radians
        """
        # Create a trajectory point with the target positions
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start =  Duration(sec=2)# Allow 2 seconds for movement

        # Create and send the goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = [point]

        self.arm_client.send_goal_async(goal_msg)

    def send_gripper_command(self, position: float) -> None:
        """
        Send a command to the gripper to open or close.

        Args:
            position (float): Position value for gripper (0.0 for open, -0.7 for closed)
        """
        # Create and send the gripper command
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 4.0

        self.gripper_client.send_goal_async(goal_msg)

    def control_loop_callback(self) -> None:
        """
        Execute a pick-and-place routine.

        This method performs the following sequence:
        1. Move the arm to the red cube, pause there and close the gripper
        2. Move the arm to the red bin through an intermediate position, then open the gripper
        3. Move the arm back to the home position
        4. Move the arm to the blue cube through a middle position, then close the gripper
        5. Move the arm to the blue bin through an intermediate position, then open the gripper
        6. Move the arm back to the home position
        """

        # # 1
        # self.get_logger().info('Reaching the red cube')
        # self.send_arm_command(self.red_cube_pose)
        # time.sleep(4.5)  # Wait for arm to reach target (2.5s)
        # self.get_logger().info('Closing the gripper')
        # self.send_gripper_command(-0.6)
        # time.sleep(2.5)

        # # 2
        # self.get_logger().info('Passing trough an intermidiate position')
        # self.send_arm_command(self.middle_pose)
        # time.sleep(3.5)
        # self.get_logger().info('Reaching the red bin')
        # self.send_arm_command(self.red_bin_pose)
        # time.sleep(5.5)
        # self.get_logger().info('Opening the gripper')
        # self.send_gripper_command(0.0)
        # time.sleep(2.5)

        # # 3 
        # self.get_logger().info('Back to home position!')
        # self.send_arm_command(self.home_pos)
        # time.sleep(4.5)

        # 4 
        self.get_logger().info("Reaching the blue cube")
        self.send_arm_command(self.middle_blue_pose)
        time.sleep(3.5)
        self.send_arm_command(self.blue_cube_pose)
        time.sleep(5)
        self.send_gripper_command(-0.6)
        time.sleep(3.5)

        # 5 
        # self.get_logger().info("Reaching the blue bin")
        # self.send_arm_command(self.middle_pose)
        # time.sleep(4.0)
        # self.send_arm_command(self.blue_bin_pose)
        # time.sleep(4.5)
        # self.send_gripper_command(0.0)
        # time.sleep(3.0)

        # # 6
        # self.get_logger().info('Back to home position!')
        # self.send_arm_command(self.home_pos)
        # time.sleep(3.5)

        # Final pause 
        time.sleep(1.0)
        self.timer.cancel()
        self.get_logger().info('Shutting down the node. Bye!')
        self.destroy_node()          
        rclpy.shutdown() 
        

def main(args=None):

    rclpy.init(args=args)
    pick_and_place = PickAndPlace()

    rclpy.spin(pick_and_place)

if __name__ == '__main__':
    main()