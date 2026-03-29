#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_pick_and_place_pipeline");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
    public:
        MTCTaskNode(const rclcpp::NodeOptions& options);
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
        void doTask();
        void setupPlanningScene();

    private:
        mtc::Task createTask();
        mtc::Task task_;
        rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
    return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
    moveit_msgs::msg::CollisionObject red_cube;
    red_cube.header.frame_id = "world";
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

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObjects({ red_cube, blue_cube, red_bin });
}

void MTCTaskNode::doTask()
{
    task_ = createTask();

    try
    {
        {
            task_.init();
        }
    }
    catch(mtc::InitStageException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER,e);
        return;
    }

    if (!task_.plan(5))
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
    }
    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
    }

    return;
    
}

mtc::Task MTCTaskNode::createTask()
{
    mtc::Task task;
    task.stages()->setName("demo task");
    task.loadRobotModel(node_);
    const auto& arm_group_name = "arm";
    const auto& hand_group_name = "gripper";
    const auto& hand_frame = "link&_flange";

    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));

    return task;
    
}

int main(int argc, char**argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
    });

    mtc_task_node->setupPlanningScene();
    mtc_task_node->doTask();

    spin_thread->join();
    rclcpp::shutdown();
    return 0;
}