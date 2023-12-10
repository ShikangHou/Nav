#ifndef _TEST_PLUGIN_H
#define _TEST_PLUGIN_H

#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <boost/shared_ptr.hpp>

class TestPlanner
{
private:
    bool transformStartPose(void);

    boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_; // 保存插件的地址

    std::vector<geometry_msgs::PoseStamped> *planner_plan_; // 保存路径
    geometry_msgs::PoseStamped goal_pose_;                  // 目标位姿
    geometry_msgs::PoseStamped start_pose_;                 // 初始位姿
    geometry_msgs::TransformStamped start_transform_;       // 初始tf位姿
    costmap_2d::Costmap2DROS *costmap_;
    std::string global_planner_;

    ros::NodeHandle nh_;
    tf2_ros::Buffer &tf_;
    ros::Subscriber make_plan_;

public:
    TestPlanner(ros::NodeHandle &nh,tf2_ros::Buffer &tf);
    ~TestPlanner();
    void setGoal(const geometry_msgs::PoseStamped::ConstPtr &goal);
};

#endif
