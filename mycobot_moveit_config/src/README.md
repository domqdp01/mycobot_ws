## Pick and Place Action Client Node

This ROS2 node controls a robotic arm and gripper to perform a single pick-and-place operation using **Action Clients**.  

It works as follows:

- The node creates two **Action Clients**:
  1. One for the **arm trajectory** (`FollowJointTrajectory`) to move the robot arm to specific joint positions.
  2. One for the **gripper** (`GripperCommand`) to open or close the gripper.

- The pick-and-place sequence:
  1. Move the arm to the **red cube** position using the arm Action Client.
  2. Close the gripper to pick up the cube using the gripper Action Client.
  3. Move through an **intermediate position**, reach the **red bin**, and open the gripper to release the cube.
  4. Return the arm to the **home position**.
  5. Move the arm to the **blue cube** through an intermediate position and close the gripper to pick it up.
  6. Move to the **blue bin**, release the cube, and return the arm to the **home position**.

> **Note:** Intermediate positions are introduced to ensure a safe trajectory and prevent the robot arm from colliding with the bins.


This approach allows precise control of both the arm and the gripper using ROS2 Actions without blocking the main execution thread.

![Pick and Place Demo](demos/pick_and_place_service_action.gif)