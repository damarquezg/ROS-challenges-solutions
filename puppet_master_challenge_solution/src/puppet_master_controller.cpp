/**
 * puppet_master_controller.cpp
 * author DMG
 *
 * Implementation of the puppet_master_controller action server
 * puppet_master_challenge_solution ROS package
 */

#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

#include <puppet_master_challenge_solution/puppet_master_controller.h>

using std::string;

MoveActionServer::MoveActionServer(string name) :
        as_(nh_, name, boost::bind(&MoveActionServer::executeCB, this, _1), false),
        action_name_(name)
{
    as_.start();

    // Subscribe to turtle1/pose topic
    pose_sub_ = nh_.subscribe("turtle1/pose", 1, &MoveActionServer::poseCallback, this);

    // Publish to turtle1/cmd_vel topic
    controller_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

    // Controller tuning parameters
    threshold_ = 0.1;
    epsilon_ = 1.0;
}

MoveActionServer::~MoveActionServer(void)
{
}

void MoveActionServer::executeCB(const puppet_master_challenge::MoveGoalConstPtr &goal)
{
    ros::Rate r(1);
    bool success = true; // Start optimistically
    feedback_.pose = current_pose_;

    auto ds = goal->destination;
    ROS_INFO("Navigating to destination: (x = %f, y = %f)", ds.x, ds.y);
    as_.publishFeedback(feedback_);

    while (!closeEnough(ds))
        {
            ROS_INFO_THROTTLE(1, "Robot's pose: (x = %f, y = %f)", current_pose_.x, current_pose_.y);
            geometry_msgs::Twist controls_msg = calculateControlInputs(ds);
            ROS_INFO_THROTTLE(1, "Calculated control inputs: (u = %f, w = %f)", controls_msg.linear.x, controls_msg.angular.z);
            controller_pub_.publish(controls_msg);
        }
    feedback_.pose = current_pose_;

    if(success)
    {
        ROS_INFO("%s action succeeded!", action_name_.c_str());
        as_.setSucceeded();
    }
    
}
// This function calculate the distance between the target pose and the current 
// robot pose and return a boolean when the robot "reach" the target
bool MoveActionServer::closeEnough(turtlesim::Pose target)
{
    float norm = std::sqrt(std::pow((target.x - current_pose_.x), 2) + std::pow((target.y - current_pose_.y), 2));

    return (norm < threshold_);
}

// A callback function to obtain the current pose of the robot
void MoveActionServer::poseCallback(const turtlesim::Pose &pose_msg) {

    current_pose_.x = pose_msg.x;
    current_pose_.y = pose_msg.y;
    current_pose_.theta = pose_msg.theta;
}

// Given a destination [x_d y_d], this function determines the robot's linear, "u", 
// and angular, "w", velocities using feedback linearization where "epsilon" is a small 
// positive constant parameter
// (c.f. Control Theory for Robotics: https://doi.org/10.1007/3-540-45000-9_8)
geometry_msgs::Twist MoveActionServer::calculateControlInputs(turtlesim::Pose target_pose)
{
    float vx = target_pose.x - current_pose_.x;
    float vy = target_pose.y - current_pose_.y;
    float theta = current_pose_.theta;

    float u = vx*std::cos(theta) + vy*std::sin(theta);
    float w = (1.0/epsilon_)*(-vx*std::sin(theta) + vy*std::cos(theta));

    //TODO: It turns too fast. We should add control input saturation constraints. 

    geometry_msgs::Twist controls_msg;
    controls_msg.linear.x = u;
    controls_msg.angular.z = w;

    return controls_msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move");

    MoveActionServer move(ros::this_node::getName());
    ros::spin();

    return 0;
}
