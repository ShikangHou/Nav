#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <vector>

using namespace std;

bool isArriveGoal = true;

vector<array<float, 3>> poses = {{2, 3, 0}, {1.6, -2.1, 3}, {-1.5, -1.9, 1.5}};

void done_cb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        isArriveGoal = true;
        ROS_INFO("arrive Goal");
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "send_goal_node");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base", true);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";

    vector<array<float, 3>>::iterator iter = poses.begin();

    while (!client.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    while (true)
    {
        if (isArriveGoal)
        {
            goal.target_pose.header.stamp = ros::Time::now();
            auto &pose = *iter;
            goal.target_pose.pose.position.x = pose[0];
            goal.target_pose.pose.position.y = pose[1];
            goal.target_pose.pose.position.z = 0;
            tf2::Quaternion quat;
            quat.setRPY(0, 0, pose[2]);
            goal.target_pose.pose.orientation.w = quat.getW();
            goal.target_pose.pose.orientation.x = quat.getX();
            goal.target_pose.pose.orientation.y = quat.getY();
            goal.target_pose.pose.orientation.z = quat.getZ();
            client.sendGoal(goal, &done_cb);
            isArriveGoal = false;
            iter++;
            if (iter == poses.end())
                iter = poses.begin();
            ROS_INFO("send Goal");
        }
        ros::Duration(0.5).sleep();
        ROS_INFO("waiting arrive");
    }
    ros::spin();

    return 0;
}
