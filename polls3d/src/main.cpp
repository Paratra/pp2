#include <ros/ros.h>
#include <polls3d/ming_polls.h>


int main(int argc, char **argv)
{
    ROS_INFO("DingDingDingDing");
    ros::init(argc, argv, "ming_polls_node");
    ros::NodeHandle nh("~");
    Ming_Polls Ming_Polls(nh);
    ros::spin();
    return 0;
}
