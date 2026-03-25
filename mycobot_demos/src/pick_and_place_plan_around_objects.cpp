#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>

void moveGripper(double position, double max_effort = 4.0)
{
    using GripperCommand = control_msgs::action::GripperCommand;

    auto gripper_node = std::make_shared<rclcpp::Node>("gripper_client_node");
    auto client = rclcpp_action::create_client<GripperCommand>(
        gripper_node, "gripper_action_controller/gripper_cmd");

    client->wait_for_action_server();

    auto goal = GripperCommand::Goal();
    goal.command.position = position;
    goal.command.max_effort = max_effort;

    auto future = client->async_send_goal(goal);
    
    // Timeout di 3 secondi per l'invio del goal
    if (rclcpp::spin_until_future_complete(
            gripper_node, future,
            std::chrono::seconds(3)) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(gripper_node->get_logger(), "Gripper goal timeout, continuing anyway");
        return;
    }

    auto result_future = client->async_get_result(future.get());
    
    // Timeout di 3 secondi per il risultato
    rclcpp::spin_until_future_complete(
        gripper_node, result_future,
        std::chrono::seconds(3));
}

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
    arm_group_interface.setPlanningTime(20.0);
    arm_group_interface.setNumPlanningAttempts(50);
    arm_group_interface.setMaxVelocityScalingFactor(1.0);
    arm_group_interface.setMaxAccelerationScalingFactor(1.0);
    
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

    // ---------------------------------------------- //
    // --- PICK & PLACE RED CUBE INTO THE RED BIN --- // 
    // ---------------------------------------------- //

    // --- STEP 1: Open the gripper ---
    RCLCPP_INFO(logger, "Opening the gripper");
    moveGripper(0.0);

    // --- STEP 2: Move the arm to the red cube --- //
    auto const target = [&node]{
       geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "world";
        target.header.stamp = node->now();
        target.pose.position.x = 0.233;
        target.pose.position.y = 0.174;
        target.pose.position.z = 0.052;
        target.pose.orientation.x = -0.763;
        target.pose.orientation.y = -0.108;
        target.pose.orientation.z = -0.052;
        target.pose.orientation.w = 0.635; 
        return target;
    }();
    
    arm_group_interface.setPoseTarget(target);

    auto const [success, plan_rc] = [&arm_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg_rc;
        auto const ok = static_cast<bool>(arm_group_interface.plan(msg_rc));
        return std::make_pair(ok, msg_rc);
    }();

    if (success)
    {
        RCLCPP_INFO(logger, "Moving to the red cube!");
        arm_group_interface.execute(plan_rc);
        rclcpp::sleep_for(std::chrono::seconds(3));

        // --- STEP 3: Attach the cube to the gripper --- //

        arm_group_interface.attachObject("red_cube", "link6_flange",
        {"gripper_left1", "gripper_left2", "gripper_left3",
        "gripper_right1", "gripper_right2", "gripper_right3"});
        RCLCPP_INFO(logger, "Red cube attached to gripper");

        // --- STEP 4: Closing the gripper --- //

        RCLCPP_INFO(logger, "Closing the gripper");        
        moveGripper(-0.6);
        rclcpp::sleep_for(std::chrono::seconds(3));

        // --- STEP 5: Place into the red bin --- //

        geometry_msgs::msg::PoseStamped red_bin_target;
        red_bin_target.header.frame_id = "world";
        red_bin_target.header.stamp = node->now();
        red_bin_target.pose.position.x = -0.117;
        red_bin_target.pose.position.y = 0.297;
        red_bin_target.pose.position.z = 0.170;
        red_bin_target.pose.orientation.x = -0.283;
        red_bin_target.pose.orientation.y = 0.296;
        red_bin_target.pose.orientation.z = 0.052;
        red_bin_target.pose.orientation.w = 0.911; 

        arm_group_interface.setPoseTarget(red_bin_target);
        moveit::planning_interface::MoveGroupInterface::Plan plan_rb;

        if (arm_group_interface.plan(plan_rb) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Moving to the red bin");
            arm_group_interface.execute(plan_rb);
            rclcpp::sleep_for(std::chrono::seconds(3));

            // --- STEP 6: Opening the gripper --- //

            arm_group_interface.detachObject("red_cube");
            rclcpp::sleep_for(std::chrono::milliseconds(500));

            planning_scene_interface.removeCollisionObjects({"red_cube"});
            rclcpp::sleep_for(std::chrono::milliseconds(300));

            RCLCPP_INFO(logger, "Opening the gripper");
            moveGripper(0.0);
        }
    }
    else
    {
        RCLCPP_ERROR(logger, "Red cube pick planning failed! Aborting pipeline.");
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    // ------------------------------------------------ //
    // --- PICK & PLACE BLUE CUBE INTO THE BLUE BIN --- // 
    // ------------------------------------------------ //

    // STEP 1: Moving the arm to the blue cube //

    geometry_msgs::msg::PoseStamped blue_cube_target;
    blue_cube_target.header.frame_id = "world";
    blue_cube_target.header.stamp = node->now();
    blue_cube_target.pose.position.x = 0.103;
    blue_cube_target.pose.position.y = 0.192;
    blue_cube_target.pose.position.z = 0.06;
    blue_cube_target.pose.orientation.x = -0.720;
    blue_cube_target.pose.orientation.y = -0.028;
    blue_cube_target.pose.orientation.z = 0.041;
    blue_cube_target.pose.orientation.w = 0.692;

    arm_group_interface.setPoseTarget(blue_cube_target);

    auto const [success_bc, plan_bc] = [&arm_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg_bc;
        auto const ok_bc = static_cast<bool>(arm_group_interface.plan(msg_bc));
        return std::make_pair(ok_bc, msg_bc);
    }();

    if (success_bc)
    {
        RCLCPP_INFO(logger, "Moving to the blue cube");
        arm_group_interface.execute(plan_bc);
        rclcpp::sleep_for(std::chrono::seconds(3));

        // --- STEP 2: Attach the blue cube to the gripper --- //

        arm_group_interface.attachObject("blue_cube", "link6_flange",
        {"gripper_left1", "gripper_left2", "gripper_left3",
        "gripper_right1", "gripper_right2", "gripper_right3"});
        RCLCPP_INFO(logger, "Blue cube attached to gripper");

        // --- STEP 3: Closing the gripper --- //

        RCLCPP_INFO(logger, "Closing the gripper");
        moveGripper(-0.6);
        rclcpp::sleep_for(std::chrono::seconds(3));

        // --- STEP 4: Place the blue cube into the blue bin --- //

        geometry_msgs::msg::PoseStamped blue_bin_target;
        blue_bin_target.header.frame_id = "world";
        blue_bin_target.header.stamp = node->now();
        blue_bin_target.pose.position.x = 0.113;
        blue_bin_target.pose.position.y = 0.286;
        blue_bin_target.pose.position.z = 0.155;
        blue_bin_target.pose.orientation.x = -0.216;
        blue_bin_target.pose.orientation.y = 0.029;
        blue_bin_target.pose.orientation.z = -0.061;
        blue_bin_target.pose.orientation.w = 0.974; 

        arm_group_interface.setPoseTarget(blue_bin_target);
        moveit::planning_interface::MoveGroupInterface::Plan plan_bb;

        if (arm_group_interface.plan(plan_bb) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Moving to the blue bin");
            arm_group_interface.execute(plan_bb);
            rclcpp::sleep_for(std::chrono::seconds(3));

            // --- STEP 5: Opening the gripper --- //

            arm_group_interface.detachObject("blue_cube");
            rclcpp::sleep_for(std::chrono::milliseconds(500));

            planning_scene_interface.removeCollisionObjects({"blue_cube"});
            rclcpp::sleep_for(std::chrono::milliseconds(300));

            RCLCPP_INFO(logger, "Opening the gripper");
            moveGripper(0.0);
        }
    }
    else
    {
        RCLCPP_ERROR(logger, "Blue cube pick planning failed! Aborting pipeline.");
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    // ------------------------------------ //
    // --- MOVING BACK TO HOME POSITION --- // 
    // ------------------------------------ //

    geometry_msgs::msg::PoseStamped home;
    home.header.frame_id = "world";
    home.header.stamp = node->now();
    home.pose.position.x = 0.061;
    home.pose.position.y = 0.080;
    home.pose.position.z = 0.410;
    home.pose.orientation.x = 0.004;
    home.pose.orientation.y = 0.000;
    home.pose.orientation.z = 0.000;
    home.pose.orientation.w = 1.000; 

    arm_group_interface.setPoseTarget(home);

    auto const [success_home, plan_home] = [&arm_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg_home;
        auto const ok_home = static_cast<bool>(arm_group_interface.plan(msg_home));
        return std::make_pair(ok_home, msg_home);
    }();

    if (success_home)
    {
        RCLCPP_INFO(logger, "Moving back to home position...");
        arm_group_interface.execute(plan_home);
        rclcpp::sleep_for(std::chrono::seconds(3));        
    }
    else
    {
        RCLCPP_ERROR(logger, "Failed to plan home position. Aborting pipeline.");
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    RCLCPP_INFO(logger, "Pick and place pipeline completed successfully. Shutting down the node. Bye!");
    rclcpp::shutdown();
    spinner.join();
    return 0;
}