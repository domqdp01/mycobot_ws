#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto const node = std::make_shared<rclcpp::Node>(
        "pick_and_place_plan_around_objects",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );
        
    auto const logger = rclcpp::get_logger("pick_and_place_plan_around_objects");
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });
    
    using moveit::planning_interface::MoveGroupInterface;
    auto arm_group_interface = MoveGroupInterface(node, "arm");
    
    arm_group_interface.setPlanningPipelineId("ompl");
    arm_group_interface.setPlannerId("RRTConnectkConfigDefault");
    arm_group_interface.setPlanningTime(1.0);
    arm_group_interface.setMaxVelocityScalingFactor(1.0);
    arm_group_interface.setMaxAccelerationScalingFactor(1.0);
    
    RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
    RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
    RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());
    
    auto gripper_group_interface = MoveGroupInterface(node, "gripper");
      
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{ node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, arm_group_interface.getRobotModel() };
    
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();
    
    //auto const draw_trajectory_tool_path = [&moveit_visual_tools, jmg = arm_group_interface.getRobotModel()->getJointModelGroup("arm")](
      //  auto const trajectory) {moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);};
        
    auto const red_cube_pose = [&node]{
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "base_link";
        msg.header.stamp = node->now();
        msg.pose.position.x = 0.061;
        msg.pose.position.y = -0.142;
        msg.pose.position.z = 0.168;
        
        msg.pose.orientation.x = 0.704;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 0.710;
        return msg;
    }();
    
    auto const blue_cube_pose = [&node]{
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "base_link";
        msg.header.stamp = node->now();
        msg.pose.position.x = 0.105;
        msg.pose.position.y = 0.182;
        msg.pose.position.z = 0.091;
        
        msg.pose.orientation.x = 0.713;
        msg.pose.orientation.y = -0.701;
        msg.pose.orientation.z = 0.010;
        msg.pose.orientation.w = 0.023;
        return msg;
    }();
    
    auto const red_bin_pose = [&node]{
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "base_link";
        msg.header.stamp = node->now();
        msg.pose.position.x = -0.108;
        msg.pose.position.y = 0.269;
        msg.pose.position.z = 0.174;
        
        msg.pose.orientation.x = 0.846;
        msg.pose.orientation.y = -0.171;
        msg.pose.orientation.z = -0.246;
        msg.pose.orientation.w = -0.441;
        return msg;
    }();
    
    auto const blue_bin_pose = [&node]{
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "base_link";
        msg.header.stamp = node->now();
        msg.pose.position.x = 0.105;
        msg.pose.position.y = 0.275;
        msg.pose.position.z = 0.183;
        
        msg.pose.orientation.x = 0.939;
        msg.pose.orientation.y = -0.074;
        msg.pose.orientation.z = 0.026;
        msg.pose.orientation.w = -0.334;
        return msg;
    }();
    
    auto const home = [&node]{
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "base_link";
        msg.header.stamp = node->now();
        msg.pose.position.x = 0.061;
        msg.pose.position.y = 0.080;
        msg.pose.position.z = 0.410;
        
        msg.pose.orientation.x = 0.004;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 1.0;
        return msg;
    }();
    
    auto move_to_pose = [&](const geometry_msgs::msg::Pose& pose){
        arm_group_interface.setPoseTarget(pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success)
        {
            //draw_trajectory_tool_path(plan.trajectory_);
            arm_group_interface.execute(plan);
        }
        else
        {
            RCLCPP_ERROR(logger, "PLANNING FAILED!!!");
        }
        
    };
    
    auto open_gripper = [&]()
    {
        gripper_group_interface.setNamedTarget("open");
        gripper_group_interface.move();
    };
    
    auto close_gripper = [&]()
    {
        gripper_group_interface.setNamedTarget("close");
        gripper_group_interface.move();
    };
    
    auto pick_and_place = [&](geometry_msgs::msg::Pose pick,
                            geometry_msgs::msg::Pose place,
                            std::string object_id)
    {
        move_to_pose(pick);
        close_gripper();
        
        arm_group_interface.attachObject(object_id);
        rclcpp::sleep_for(std::chrono::seconds(3));
        
        move_to_pose(place);
        open_gripper();        
        arm_group_interface.detachObject(object_id);
        rclcpp::sleep_for(std::chrono::seconds(3));

        move_to_pose(home.pose);
        rclcpp::sleep_for(std::chrono::seconds(3));

    };

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    auto addRedCube = [frame_id = arm_group_interface.getPlanningFrame(), &node, &logger, &planning_scene_interface]() {
        RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());

        moveit_msgs::msg::CollisionObject red_cube;
        red_cube.header.frame_id = frame_id;
        red_cube.header.stamp = node->now();
        red_cube.id = "red_cube";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.02;
        primitive.dimensions[primitive.BOX_Y] = 0.02;
        primitive.dimensions[primitive.BOX_Z] = 0.02;

        geometry_msgs::msg::Pose red_cube_pose;
        red_cube_pose.position.x = 0.25;
        red_cube_pose.position.y = 0.16;
        red_cube_pose.position.z = -0.02;
        red_cube_pose.orientation.x = 0.0;
        red_cube_pose.orientation.y = 0.0;
        red_cube_pose.orientation.z = 0.0;
        red_cube_pose.orientation.w = 1.0;

        red_cube.primitives.push_back(primitive);
        red_cube.primitive_poses.push_back(red_cube_pose);
        red_cube.operation = red_cube.ADD;

        planning_scene_interface.applyCollisionObject(red_cube);
    };

    // Eseguire la lambda
    addRedCube();
    
    auto addRedBin = [frame_id = arm_group_interface.getPlanningFrame(), &node, &logger, &planning_scene_interface]() {
    RCLCPP_INFO(logger, "Adding red bin as a single collision object (frame: %s)", frame_id.c_str());

    moveit_msgs::msg::CollisionObject red_bin;
    red_bin.header.frame_id = frame_id;
    red_bin.header.stamp = node->now();
    red_bin.id = "red_bin";

    // Unico parallelepipedo per tutto il cestino
    shape_msgs::msg::SolidPrimitive bin_shape;
    bin_shape.type = bin_shape.BOX;
    bin_shape.dimensions = {0.5, 0.05, 0.10};  // larghezza X, profondità Y, altezza Z

    geometry_msgs::msg::Pose bin_pose;
    bin_pose.position.x = 0.0;   // centro in X
    bin_pose.position.y = 0.30;    // centro in Y
    bin_pose.position.z = 0.05;    // metà altezza del parallelepipedo (0.10/2)
    bin_pose.orientation.w = 1.0;  // nessuna rotazione

    red_bin.primitives.push_back(bin_shape);
    red_bin.primitive_poses.push_back(bin_pose);

    red_bin.operation = red_bin.ADD;

    planning_scene_interface.applyCollisionObject(red_bin);
    RCLCPP_INFO(logger, "Red bin added successfully as single box!");
    };
    addRedBin();

    pick_and_place(red_cube_pose.pose, red_bin_pose.pose, "red_cube");

    return 0;
    
}