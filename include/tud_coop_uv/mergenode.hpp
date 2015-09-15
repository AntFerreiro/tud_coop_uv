#ifndef MERGENODE_HPP
#define MERGENODE_HPP

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tud_coop_uv/SetController.h"



class MergeNode
{
public:
    MergeNode();
    ros::NodeHandle nh;

    void tracking_callback(const geometry_msgs::Twist& cmd_vel);
    void coverage_callback(const geometry_msgs::Twist& cmd_vel);
    void joy_callback(const geometry_msgs::Twist& cmd_vel);

    bool set_controller_callback(tud_coop_uv::SetController::Request& request,
                                 tud_coop_uv::SetController::Response& response);


private:
    ros::Subscriber m_tracking_sub;
    ros::Subscriber m_coverage_sub;
    ros::Subscriber m_joy_sub;

    ros::Publisher m_cmd_vel_pub;

    geometry_msgs::Twist m_cmd_vel_tracking;
    geometry_msgs::Twist m_cmd_vel_coverage;
    geometry_msgs::Twist m_cmd_vel_joy;

    ros::ServiceServer m_set_controller_srv;
};

#endif // MERGENODE_HPP
