#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>


using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    robot_state::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;

    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose target_pose;
    //target_pose = start_pose;

    target_pose.position.x = 0.307047139963;
    target_pose.position.y = 0.0900418054465;
    target_pose.position.z = 0.639448946626;
    waypoints.push_back(target_pose);


    

    // target_pose.position.z -= 0.2;
    // waypoints.push_back(target_pose); // down

    // target_pose.position.y -= 0.2;
    // waypoints.push_back(target_pose); // right
    
    // target_pose.position.x += 0.2;
    // target_pose.position.z +=0.2;
    // waypoints.push_back(target_pose); // forward

    // target_pose.position.x -= 0.2;
    // target_pose.position.z -=0.2;
    // waypoints.push_back(target_pose); // backwoard
    // target_pose.position.z += 0.2;
    // target_pose.position.y += 0.2;
    // target_pose.position.x -= 0.2;
    // waypoints.push_back(target_pose); // up and left

    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    moveit::planning_interface::MoveGroupInterface::Plan goal_plan;
    goal_plan.trajectory_ = trajectory;

    bool success = (move_group.execute(goal_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    ROS_INFO_NAMED("test", "The execute %s", success ? "SUCCESSED" : "FAILED");

    // if(success){
    //     move_group.setPoseTarget(start_pose);
    //     move_group.move();
    // }
}
