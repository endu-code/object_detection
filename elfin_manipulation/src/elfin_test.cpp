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

int main(int argc, char **argv)
{
    const std::string node_name = "elfin_test";
    ros::init(argc, argv, node_name);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh_("~");

    // Robot Model Loader:
    const std::string PLANNING_ARM = "elfin_arm_gripper_small";
    //const std::string PLANNING_GRIPPER_125 = "elfin_gripper_125";
    //const std::string PLANNING_ARM = "elfin_arm";

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(PLANNING_ARM);
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_ARM);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    addCollisionObjects(planning_scene_interface);
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

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
    moveit_visual_tools::MoveItVisualTools visual_tools("elfin_base_link", "/visualization_marker_array");
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Paoluzzo Elfin Test", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    visual_tools.prompt("Press Next to Continue!");

    //Creating a Motion plan request for elfin_arm

    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rvt::GREEN);
    visual_tools.trigger();

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped current;

    pose.header.frame_id = "elfin_base";
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.25;
    pose.pose.position.z = 0.67788;
    pose.pose.orientation.w =1;
    pose.pose.orientation.x=1;
    pose.pose.orientation.y=1;
    pose.pose.orientation.z=1;

    current = move_group.getCurrentPose("tcp1");

    
    ROS_WARN_STREAM("Current Robot Position: Positions: X:" << current.pose.position.x << ", Y:" << current.pose.position.y <<", Z:" << current.pose.position.z << ", \n Orientation: W: " << current.pose.orientation.w<< ", X: " << current.pose.orientation.x<< ", Y: " << current.pose.orientation.y<< ", Z:" << current.pose.orientation.z);

    std::vector<double> tolerance_pose(3, 0.001);
    std::vector<double> tolerance_angle(3, 0.001);

    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("tcp1", pose, tolerance_pose, tolerance_angle);

    req.group_name = PLANNING_ARM;
    req.goal_constraints.push_back(pose_goal);
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
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
    current = move_group.getCurrentPose("tcp1");

    
    ROS_WARN_STREAM("Current Robot Position: Positions: X:" << current.pose.position.x << ", Y:" << current.pose.position.y <<", Z:" << current.pose.position.z << ", \n Orientation: W: " << current.pose.orientation.w<< ", X: " << current.pose.orientation.x<< ", Y: " << current.pose.orientation.y<< ", Z:" << current.pose.orientation.z);


    visual_tools.prompt("2. Press next in the RVizVisualToolsGUI to start demo");

    // to use joint angles in deg instead of rad multiply the deg angle with deg2rad

    robot_state::RobotState goal_state(robot_model);

    std::vector<double> joint_values = {90 * deg2rad, 45 * deg2rad, 90 * deg2rad, 0 * deg2rad, 90 * deg2rad, 0 * deg2rad};
    goal_state.setJointGroupPositions(joint_model_group, joint_values);

    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);

    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);

    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Couldn't compute plan succesfully");
        return 0;
    }

    res.getMessage(response);
    display_trajectory.trajectory.push_back(response.trajectory);

    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher.publish(display_trajectory);

    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rvt::GREEN);
    visual_tools.publishAxisLabeled(pose.pose, "goal2");
    visual_tools.publishText(text_pose, "joint Space Goal (2)", rvt::WHITE, rvt::XLARGE);

    plan.trajectory_ = response.trajectory;
    move_group.execute(plan);
    current = move_group.getCurrentPose("tcp1");
    ROS_WARN_STREAM("Current Robot Position: Positions: X:" << current.pose.position.x << ", Y:" << current.pose.position.y <<", Z:" << current.pose.position.z << ", \n Orientation: W: " << current.pose.orientation.w<< ", X: " << current.pose.orientation.x<< ", Y: " << current.pose.orientation.y<< ", Z:" << current.pose.orientation.z);



    visual_tools.prompt("3. Press Next to continue");

    return 0;
}



void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(5);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table";
  collision_objects[0].header.frame_id = "elfin_base";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.75;
  collision_objects[0].primitives[0].dimensions[1] = 1.5;
  collision_objects[0].primitives[0].dimensions[2] = 1.0;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.375+0.15;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = -0.52;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "wall_right";
  collision_objects[1].header.frame_id = "elfin_base";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.75;
  collision_objects[1].primitives[0].dimensions[1] = 0.03;
  collision_objects[1].primitives[0].dimensions[2] = 0.7;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.375+0.15;
  collision_objects[1].primitive_poses[0].position.y = 0.75-(0.03/2)    ;
  collision_objects[1].primitive_poses[0].position.z = 0.32;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "elfin_base";
  collision_objects[2].id = "wall_left";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.75;
  collision_objects[2].primitives[0].dimensions[1] = 0.03;
  collision_objects[2].primitives[0].dimensions[2] = 0.7;

  /* Define the pose of the table. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.375+0.15;
  collision_objects[2].primitive_poses[0].position.y = -0.75+(0.03/2);
  collision_objects[2].primitive_poses[0].position.z = 0.32;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

    // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[3].header.frame_id = "elfin_base";
  collision_objects[3].id = "wall_left_machine";

  /* Define the primitive and its dimensions. */
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 0.3;
  collision_objects[3].primitives[0].dimensions[1] = 0.03;
  collision_objects[3].primitives[0].dimensions[2] = 1.4;

  /* Define the pose of the table. */
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0;
  collision_objects[3].primitive_poses[0].position.y = -0.75+(0.03/2);
  collision_objects[3].primitive_poses[0].position.z = -0.02;
  // END_SUB_TUTORIAL

  collision_objects[3].operation = collision_objects[3].ADD;

      // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[4].header.frame_id = "elfin_base";
  collision_objects[4].id = "wall_right_machine";

  /* Define the primitive and its dimensions. */
  collision_objects[4].primitives.resize(1);
  collision_objects[4].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[4].primitives[0].dimensions.resize(3);
  collision_objects[4].primitives[0].dimensions[0] = 0.3;
  collision_objects[4].primitives[0].dimensions[1] = 0.03;
  collision_objects[4].primitives[0].dimensions[2] = 1.4;

  /* Define the pose of the table. */
  collision_objects[4].primitive_poses.resize(1);
  collision_objects[4].primitive_poses[0].position.x = 0;
  collision_objects[4].primitive_poses[0].position.y = 0.75-(0.03/2);
  collision_objects[4].primitive_poses[0].position.z = -0.02;
  // END_SUB_TUTORIAL

  collision_objects[4].operation = collision_objects[4].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}
