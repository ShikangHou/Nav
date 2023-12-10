#include "test_plugin.h"
#include <pluginlib/class_loader.h>

// 使用全局规划器的插件
TestPlanner::TestPlanner(ros::NodeHandle &nh,tf2_ros::Buffer &tf) : nh_(nh),tf_(tf)
{
    make_plan_ = nh_.subscribe("/move_base_simple/goal", 2, &TestPlanner::setGoal, this);
    // 定义插件的名称
    global_planner_ = std::string("hybrid_astar_planner/HybridAStarPlanner");
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner");
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    costmap_ = new costmap_2d::Costmap2DROS("/global_costmap", tf);

    std::cout << "creat the global costmap" << std::endl;

    // 获取base_link在map坐标系中的位置作为初始坐标
    transformStartPose();
    try
    {
        planner_ = bgp_loader_.createInstance(global_planner_);
        planner_->initialize(bgp_loader_.getName(global_planner_), costmap_);
    }
    catch (pluginlib::PluginlibException &ex)
    {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        exit(1);
    }
}

void TestPlanner::setGoal(const geometry_msgs::PoseStamped::ConstPtr &goal)
{
    goal_pose_.pose = goal->pose;
    goal_pose_.header = goal->header;
    transformStartPose();
    costmap_->start();
    planner_->makePlan(start_pose_, goal_pose_, *planner_plan_);
}

bool TestPlanner::transformStartPose()
{
    try
    {
        start_transform_ = tf_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    start_pose_.pose.position.x = start_transform_.transform.translation.x;
    start_pose_.pose.position.y = start_transform_.transform.translation.y;
    start_pose_.pose.position.z = start_transform_.transform.translation.z;
    start_pose_.pose.orientation.x = start_transform_.transform.rotation.x;
    start_pose_.pose.orientation.y = start_transform_.transform.rotation.y;
    start_pose_.pose.orientation.z = start_transform_.transform.rotation.z;
    start_pose_.pose.orientation.w = start_transform_.transform.rotation.w;
    return true;
}

TestPlanner::~TestPlanner()
{
    planner_.reset();
    if (costmap_)
    {
        delete costmap_;
    }
}