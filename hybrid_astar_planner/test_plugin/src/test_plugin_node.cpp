#include "test_plugin.h"
#include <tf/transform_listener.h>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"test_plugin");
    ros::NodeHandle nh;

    // buffer中记录10秒内的数据
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    TestPlanner test(nh,buffer);

    ros::spin();    
    return 0;
}
