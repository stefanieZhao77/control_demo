#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>


#include <actionlib/server/simple_action_server.h>

using namespace std;

class franka_server
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<franka_server::frankaAction>as_;
    std::string action_name;
    franka_server::frankaAction feedback_;
    franka_server::frankaAction result_;
     
    moveit::planning_interface::MoveGroupInterface *move_group;
    moveit::planning_interface::PlanningSceneInterface *group_scene_interface;
public:
    frankaAction(std::string direction, std::string name);
    
    ~frankaAction();
    void run();
    void get_cartesian_positon();
    void initial();
    void executeCB(const control_demo::frankaActionConstPtr &goal);

};


