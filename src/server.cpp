#include <ros/ros.h>

#include <actionlib/client/simple_action_server.h>

using namespace std;

class MainAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<control_demo::MainAction> as_;
    std::string action_name_;
    control_demo::MainFeedback feedback_;
    control_demo::MainResult result_;

public:
    MainAction(std::string name) : as_(nh_, name, boost::bind(&MainAction::executeCB, this, _1), false);
    action_name_(name)
    {
        as_.start();
    }
    ~MainAction(void)
    {

    }
    void executeCB(const control_demo::MainActionConstPtr &goal){
        ros::Rate(2);
        bool success = true;
        geometry_msgs::Pose target_pose;
        target_pose = goal->goal_pose;
        
        
    }
}