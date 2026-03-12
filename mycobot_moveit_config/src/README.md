## Pick and Place Action Client Node

This ROS2 node controls a robotic arm and gripper to perform a single pick-and-place operation.  
The robot moves to a red cube, picks it up with the gripper, moves to an intermediate position, places the cube into a red bin, and finally returns to the home position.  
The node automatically shuts down after completing the sequence.