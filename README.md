# MyCobot Pick and Place Project #
![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)

# MyCobot Pick and Place Project 🤖

> **Work in Progress ⚠️** – This repository is under active development.

This repository contains ROS 2 packages for controlling and simulating the **myCobot robotic arm**, with a focus on **pick and place tasks**. The project explores:

- Forward and inverse kinematics for precise robot control
- Computer vision using **OpenCV** for color and object detection
- Neural networks for advanced object recognition

Simulation allows testing algorithms without hardware, with visualization support in **RViz**.

---

## Features ✨
- ROS 2 control of myCobot
- Object and color recognition with OpenCV
- Neural network-based object classification
- Motion planning and manipulation of multiple objects
- RViz visualization of robot state
- Multi-object pick and place support
- Coordinate transformations via tf2

---

## TODO / Work in Progress 📋

- [ ] Pick and place demo with RViz
- [ ] OpenCV color detection
- [ ] Neural network object recognition

---

## Getting Started 🚀

### Prerequisites
- ROS 2 Jazzy
- Python 3
- OpenCV
- Neural network libraries (TBD)

### Installation
1. Clone the repository into your ROS 2 workspace `src` folder:

```bash
cd ~/mycobot_ws/src
git clone git@github.com:domqdp01/mycobot_ws.git
cd ~/mycobot_ws
