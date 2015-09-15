#ifndef MERGENODE_HPP
#define MERGENODE_HPP

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class MergeNode
{
public:
    MergeNode();
    ros::NodeHandle nh;

private:
    ros::Subscriber m_cmd_vel_tracking_sub;
    ros::Subscriber m_cmd_vel_coverage_sub;
    ros::Subscriber m_cmd_vel_joy_sub;

    ros::Publisher m_cmd_vel_pub;
};

#endif // MERGENODE_HPP
