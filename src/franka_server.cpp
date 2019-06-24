#include <franka_server.h>

using namespace std;

// planing
frankaAction::planning(std::string name, std::string direction)
{
    // create the server
    as_(nh_, name, boost::bind(&frankaAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start()
    }

    // define the plan of Franka
    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;
    geometry_msgs::Pose target_pose;
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> waypoints;

    if (direction == "left")
    {
        target_pose.position.y += 0.2; //left
        waypoints.push_back(target_pose);
    }
    else if (direction == "right")
    {
    }
    else if (direction == "forward")
    {
    }
    else if (direction == "back")
    {
    }
    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    moveit::planning_interface::MoveGroupInterface::Plan target_plan;
    target_plan.trajectory_ = trajectory;
    bool success = (move_group.plan(target_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}
// get pose
void frankaAction::get_cartesian_positon()
{
    geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;
}

// run

// main

// initial