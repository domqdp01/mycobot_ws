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

## Pick and Place: Plan Around Objects

This ROS2 node performs a pick-and-place operation for two cubes using **MoveIt2** for motion planning, 
with full **collision-aware trajectory planning** through a Planning Scene.

It works as follows:
- The node uses a **MoveGroupInterface** to control the robot arm, configured with the **OMPL pipeline** 
  and the **RRTConnect** planner.
- A separate **Action Client** is used for the **gripper** (`GripperCommand`), running on a dedicated node 
  in a background thread to avoid blocking the main executor.
- Before any motion, all objects in the scene are registered as **Collision Objects** in the 
  **PlanningSceneInterface**, so MoveIt2 can plan trajectories that avoid them:
  - `red_cube` — the first cube to pick
  - `blue_cube` — the second cube to pick
  - `red_bin` — an obstacle the arm must plan around

- The pick-and-place sequence:

  **Red cube:**
  1. Open the gripper.
  2. Plan and move the arm to the **red cube** pose.
  3. **Attach** the red cube to the gripper link in the planning scene.
  4. Close the gripper.
  5. Plan and move to the **red bin**, with MoveIt2 automatically routing around obstacles.
  6. **Detach** and remove the red cube from the planning scene, then open the gripper.

  **Blue cube:**
  1. Plan and move the arm to the **blue cube** pose.
  2. **Attach** the blue cube to the gripper link in the planning scene.
  3. Close the gripper.
  4. Plan and move to the **blue bin**.
  5. **Detach** and remove the blue cube from the planning scene, then open the gripper.

  **Return home:**
  6. Plan and move the arm back to the **home position**.

> **Note:** Using `attachObject()` and `detachObject()` tells MoveIt2 that the cube moves rigidly 
> with the gripper, preventing false collision detections during transport. If planning fails at any 
> step, the node logs an error and shuts down gracefully.

![Pick and Place Plan Around Objects Demo](demos/Pick_and_place_plan_around_objects.gif)
