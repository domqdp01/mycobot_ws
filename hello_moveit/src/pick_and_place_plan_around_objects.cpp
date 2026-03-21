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
    auto gripper_group_interface = MoveGroupInterface(node, "gripper");
    
    arm_group_interface.setPlanningPipelineId("ompl");
    arm_group_interface.setPlannerId("RRTConnectkConfigDefault");
    arm_group_interface.setPlanningTime(10.0);
    arm_group_interface.setNumPlanningAttempts(20);
    arm_group_interface.setMaxVelocityScalingFactor(1.0);
    arm_group_interface.setMaxAccelerationScalingFactor(1.0);

    gripper_group_interface.setGoalTolerance(0.05); 
    gripper_group_interface.setNumPlanningAttempts(10);
    gripper_group_interface.setPlanningTime(10.0);
    gripper_group_interface.setNamedTarget("closed");
    
    RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
    RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
    RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());
    
    // ------------------------- //
    // --- COLLISION OBJECTS --- //
    // ------------------------- //

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // red cube (to catch)
    moveit_msgs::msg::CollisionObject red_cube;
    red_cube.header.frame_id = "world";
    red_cube.header.stamp = node->now();
    red_cube.id = "red_cube";

    shape_msgs::msg::SolidPrimitive rcube_shape;
    rcube_shape.type = rcube_shape.BOX;
    rcube_shape.dimensions = {0.02, 0.02, 0.02};

    geometry_msgs::msg::Pose red_cube_pose;
    red_cube_pose.position.x = 0.25;
    red_cube_pose.position.y = 0.16;
    red_cube_pose.position.z = -0.02;
    red_cube_pose.orientation.x = 0.0;
    red_cube_pose.orientation.y = 0.0;
    red_cube_pose.orientation.z = 0.0;
    red_cube_pose.orientation.w = 1.0;

    red_cube.primitives.push_back(rcube_shape);
    red_cube.primitive_poses.push_back(red_cube_pose);
    red_cube.operation = red_cube.ADD;

    // Blue cube

    moveit_msgs::msg::CollisionObject blue_cube;
    blue_cube.header.frame_id = "world";
    blue_cube.header.stamp = node->now();
    blue_cube.id = "blue_cube";
    
    shape_msgs::msg::SolidPrimitive bcube_shape;
    bcube_shape.type = bcube_shape.BOX;
    bcube_shape.dimensions = {0.02, 0.02, 0.02};

    geometry_msgs::msg::Pose bcube_pose;
    bcube_pose.position.x = 0.10;
    bcube_pose.position.y = 0.18;
    bcube_pose.position.z = -0.02;
    bcube_pose.orientation.x = 0.0;
    bcube_pose.orientation.y = 0.0;
    bcube_pose.orientation.z = 0.0;
    bcube_pose.orientation.w = 1.0;

    blue_cube.primitives.push_back(bcube_shape);
    blue_cube.primitive_poses.push_back(bcube_pose);
    blue_cube.operation = blue_cube.ADD;

    // red bin (avoid the internal wall)
    moveit_msgs::msg::CollisionObject red_bin;
    red_bin.header.frame_id = "world";
    red_bin.header.stamp = node->now();
    red_bin.id = "red_bin";

    shape_msgs::msg::SolidPrimitive bin_shape;
    bin_shape.type = bin_shape.BOX;
    bin_shape.dimensions = {0.55, 0.05, 0.10};

    geometry_msgs::msg::Pose bin_pose;
    bin_pose.position.x = 0.0;   
    bin_pose.position.y = 0.30;  
    bin_pose.position.z = 0.05;   
    bin_pose.orientation.w = 1.0;

    red_bin.primitives.push_back(bin_shape);
    red_bin.primitive_poses.push_back(bin_pose);
    red_bin.operation = red_bin.ADD;

    planning_scene_interface.applyCollisionObject(red_cube);
    planning_scene_interface.applyCollisionObject(blue_cube);
    planning_scene_interface.applyCollisionObject(red_bin);
    RCLCPP_INFO(logger, "Collision objects added to planning scene");

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ----------------------------------------------
    // --- PICK & PLACE RED CUBE INTO THE RED BIN ---
    // ----------------------------------------------

    // --- STEP 1: Open the gripper ---
    RCLCPP_INFO(logger, "opening the gripper");
    gripper_group_interface.setNamedTarget("open");
    gripper_group_interface.move();

    // --- STEP 2: Move the arm to the red cube ---
    auto const target = [&node]{
       geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "world";
        target.header.stamp = node->now();
        target.pose.position.x = 0.236;
        target.pose.position.y = 0.162;
        target.pose.position.z = 0.051;
        target.pose.orientation.x = -0.531;
        target.pose.orientation.y = 0.394;
        target.pose.orientation.z = -0.527;
        target.pose.orientation.w = 0.535; 
        return target;
    }();
    
    arm_group_interface.setPoseTarget(target);

    auto const [success, plan] = [&arm_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success)
    {
        RCLCPP_INFO(logger, "Planning ok! Execution...");
        arm_group_interface.execute(plan);
        rclcpp::sleep_for(std::chrono::seconds(3));

        // --- STEP 3: Attach the cube to the gripper ---

        arm_group_interface.attachObject("red_cube", "link6_flange",
        {"gripper_left1", "gripper_left2", "gripper_left3",
        "gripper_right1", "gripper_right2", "gripper_right3"});
        RCLCPP_INFO(logger, "Object attached");
        rclcpp::sleep_for(std::chrono::seconds(2));

        // --- STEP 4: Closing the gripper ---

        RCLCPP_INFO(logger, "Closing the gripper");
        gripper_group_interface.move();
        rclcpp::sleep_for(std::chrono::seconds(3));

        // --- STEP 5: Place into the red bin ---

        geometry_msgs::msg::PoseStamped red_bin_target;
        red_bin_target.header.frame_id = "world";
        red_bin_target.header.stamp = node->now();
        red_bin_target.pose.position.x = -0.117;
        red_bin_target.pose.position.y = 0.297;
        red_bin_target.pose.position.z = 0.157;
        red_bin_target.pose.orientation.x = -0.283;
        red_bin_target.pose.orientation.y = 0.296;
        red_bin_target.pose.orientation.z = 0.052;
        red_bin_target.pose.orientation.w = 0.911; 

        arm_group_interface.setPoseTarget(red_bin_target);
        moveit::planning_interface::MoveGroupInterface::Plan plan_2;

        if (arm_group_interface.plan(plan_2) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Going to the red bin");
            arm_group_interface.execute(plan_2);
            rclcpp::sleep_for(std::chrono::seconds(3));

            // --- STEP 6: Opening the gripper ---

            arm_group_interface.detachObject("red_cube");
            rclcpp::sleep_for(std::chrono::milliseconds(500));

            planning_scene_interface.removeCollisionObjects({"red_cube"});
            rclcpp::sleep_for(std::chrono::milliseconds(300));

            gripper_group_interface.setNamedTarget("open");
            gripper_group_interface.move();
        }
    }
    else
    {
        RCLCPP_ERROR(logger, "planning failed!!!");
    }

    // ------------------------------------------------
    // --- PICK & PLACE BLUE CUBE INTO THE BLUE BIN ---
    // ------------------------------------------------

    // STEP 1: Moving the arm to the blue cube

    geometry_msgs::msg::PoseStamped blue_cube_target;
    blue_cube_target.header.frame_id = "world";
    blue_cube_target.header.stamp = node->now();
    blue_cube_target.pose.position.x = 0.104;
    blue_cube_target.pose.position.y = 0.181;
    blue_cube_target.pose.position.z = 0.06;
    blue_cube_target.pose.orientation.x = 0.518;
    blue_cube_target.pose.orientation.y = -0.486;
    blue_cube_target.pose.orientation.z = 0.505;
    blue_cube_target.pose.orientation.w = -0.490;

    arm_group_interface.setPoseTarget(blue_cube_target);

    auto const [success_bc, plan_bc] = [&arm_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg_bc;
        auto const ok_bc = static_cast<bool>(arm_group_interface.plan(msg_bc));
        return std::make_pair(ok_bc, msg_bc);
    }();

    if (success_bc)
    {
        RCLCPP_INFO(logger, "Planning ok! Execution...");
        arm_group_interface.setNumPlanningAttempts(5);
        arm_group_interface.setPlanningTime(10.0);
        arm_group_interface.execute(plan_bc);
        rclcpp::sleep_for(std::chrono::seconds(3));

        // --- STEP 2: Attach the cube to the gripper ---

        arm_group_interface.attachObject("blue_cube", "link6_flange",
        {"gripper_left1", "gripper_left2", "gripper_left3",
        "gripper_right1", "gripper_right2", "gripper_right3"});
        RCLCPP_INFO(logger, "Object attached");
        rclcpp::sleep_for(std::chrono::seconds(2));

        // --- STEP 3: Closing the gripper ---

        RCLCPP_INFO(logger, "Closing the gripper");
        gripper_group_interface.setNamedTarget("closed");
        gripper_group_interface.move();
        rclcpp::sleep_for(std::chrono::seconds(3));

        // --- STEP 4: Place into the blue bin ---

        geometry_msgs::msg::PoseStamped blue_bin_target;
        blue_bin_target.header.frame_id = "world";
        blue_bin_target.header.stamp = node->now();
        blue_bin_target.pose.position.x = 0.140;
        blue_bin_target.pose.position.y = 0.272;
        blue_bin_target.pose.position.z = 0.189;
        blue_bin_target.pose.orientation.x = -0.192;
        blue_bin_target.pose.orientation.y = 0.668;
        blue_bin_target.pose.orientation.z = -0.154;
        blue_bin_target.pose.orientation.w = 0.703; 

        arm_group_interface.setPoseTarget(blue_bin_target);
        moveit::planning_interface::MoveGroupInterface::Plan plan_2;

        if (arm_group_interface.plan(plan_2) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Going to the blue bin");
            arm_group_interface.execute(plan_2);
            rclcpp::sleep_for(std::chrono::seconds(3));

            // --- STEP 6: Opening the gripper ---

            arm_group_interface.detachObject("blue_cube");
            rclcpp::sleep_for(std::chrono::milliseconds(500));

            planning_scene_interface.removeCollisionObjects({"blue_cube"});
            rclcpp::sleep_for(std::chrono::milliseconds(300));

            gripper_group_interface.setNamedTarget("open");
            gripper_group_interface.move();
        }
    }
    else
    {
        RCLCPP_ERROR(logger, "planning failed");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}