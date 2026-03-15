## Pick and Place Action Client Node

This ROS2 node controls a robotic arm and gripper to perform a single pick-and-place operation using **Action Clients**.  

It works as follows:

- The node creates two **Action Clients**:
  1. One for the **arm trajectory** (`FollowJointTrajectory`) to move the robot arm to specific joint positions.
  2. One for the **gripper** (`GripperCommand`) to open or close the gripper.

- The pick-and-place sequence:
  1. Move the arm to the **red cube** position using the arm Action Client.
  2. Close the gripper to pick up the cube using the gripper Action Client.
  3. Move to an **intermediate position**.
  4. Move the arm to the **red bin** position.
  5. Open the gripper to release the cube.
  6. Return the arm to the **home position**.


This approach allows precise control of both the arm and the gripper using ROS2 Actions without blocking the main execution thread.

![Pick and Place Demo](demos/pick_and_place_service_action.gif)