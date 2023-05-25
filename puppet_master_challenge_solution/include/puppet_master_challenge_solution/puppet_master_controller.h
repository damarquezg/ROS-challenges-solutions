/**
 * puppet_master_controller.h
 * author DMG
 *
 * Header file for the puppet_master_controller 
 * puppet_master_challenge_solution ROS package
 */

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

#include <puppet_master_challenge/MoveAction.h>
#include <puppet_master_challenge/MoveGoal.h>

using std::string;
using puppet_master_challenge::MoveAction;
using puppet_master_challenge::MoveFeedback;
using puppet_master_challenge::MoveGoalConstPtr;

class MoveActionServer
{

public:
    MoveActionServer(string name);
    ~MoveActionServer(void);

    void executeCB(const MoveGoalConstPtr &goal);

protected:

    // Action server
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<MoveAction> as_;
    string action_name_;
    MoveFeedback feedback_;

    // Member variable for keeping track of the robot's pose
    turtlesim::Pose current_pose_;
    float threshold_;
    float epsilon_;

    // subscribers
    ros::Subscriber pose_sub_;
    // publishers
    ros::Publisher controller_pub_;

    void poseCallback(const turtlesim::Pose&);
    bool closeEnough(turtlesim::Pose);
    geometry_msgs::Twist calculateControlInputs(turtlesim::Pose);
};
