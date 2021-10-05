#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>

#include <boost/scoped_ptr.hpp>

double deg2rad = M_PI / 180;

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

int main(int argc, char** argv){
    const std::string node_name = "braccio_test_manipulation";
    ros::init(argc, argv, node_name);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh_("~");

    const std::string PLANNING_ARM = "braccio_arm";
    const std::string PANNING_GRIPPER = "braccio_gripper";

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(PLANNING_ARM);
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_ARM);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group,"home");

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    // Collision Objects and Environment

    if (!nh_.getParam("planning_plugin", planner_plugin_name))
    {
        ROS_FATAL_STREAM("Couldn't find planner plugin name!");
    }
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException &ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader" << ex.what());
    }
    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, nh_.getNamespace()))
        {
            ROS_FATAL_STREAM("Couldn't initialize planner instance");
        }
        ROS_INFO_STREAM("Using planning interface :" << planner_instance->getDescription());
    }

    catch (pluginlib::PluginlibException &ex)
    {
        const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0; i < classes.size(); ++i)
        {
            ss << classes[i] << " ";
        }
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "':" << ex.what() << std::endl
                                                             << "Available plugins" << ss.str());
    }
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link", "/visualization_marker_array");
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1;
    visual_tools.publishText(text_pose, "Endu Braccio Test", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped current;

    ROS_WARN_STREAM("Fetching robot state");
    current=move_group.getCurrentPose("tcp");
    ROS_WARN_STREAM("Current Robot Position: \n Positions:\n X: " << current.pose.position.x << ",\n Y: " << current.pose.position.y <<",\n Z: " << current.pose.position.z << ", \n Orientation:\n W: " << current.pose.orientation.w<< ",\n X: " << current.pose.orientation.x<< ",\n Y: " << current.pose.orientation.y<< ",\n Z:" << current.pose.orientation.z);

    pose.header.frame_id = "base_link";
    pose.pose.position.x = 0;
    pose.pose.position.y =-0.125298;
    pose.pose.position.z = 0.12147;
    pose.pose.orientation.w = 0.382578;
    pose.pose.orientation.x = -0.923923;
    pose.pose.orientation.y = 0.00000;
    pose.pose.orientation.z = 0.00000;


    std::vector<double> tolerance_pose(3, 0.1);
    std::vector<double> tolerance_angle(3, 0.1);

    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("wrist_roll_link", pose, tolerance_pose, tolerance_angle);

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    req.group_name = PLANNING_ARM;
    req.goal_constraints.push_back(pose_goal);

    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

    ROS_WARN_STREAM("Compute plan to goal Position: \n Positions: \n X: " << pose.pose.position.x << ",\n Y: " << pose.pose.position.y <<",\n Z: " << pose.pose.position.z << ",\n Orientation:\n W: " << pose.pose.orientation.w<< ",\n X: " << pose.pose.orientation.x<< ",\n Y: " << pose.pose.orientation.y<< ",\n Z:" << pose.pose.orientation.z);

    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Couldn't compute plan successfully");
        return 0;
    }
    ros::Publisher display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();

    display_publisher.publish(display_trajectory);

    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rvt::GREEN);
    visual_tools.publishAxisLabeled(pose.pose, "goal_1");
    visual_tools.publishText(text_pose, "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    ROS_ERROR("pre execute");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = response.trajectory;
    move_group.execute(plan);
    current = move_group.getCurrentPose("tcp");

    
    ROS_WARN_STREAM("Current Robot Position: Positions: X:" << current.pose.position.x << ", Y:" << current.pose.position.y <<", Z:" << current.pose.position.z << ", \n Orientation: W: " << current.pose.orientation.w<< ", X: " << current.pose.orientation.x<< ", Y: " << current.pose.orientation.y<< ", Z:" << current.pose.orientation.z);


    visual_tools.prompt("2. Press next in the RVizVisualToolsGUI to start demo");

}

