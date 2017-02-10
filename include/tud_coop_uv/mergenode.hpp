#ifndef MERGENODE_HPP
#define MERGENODE_HPP

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "tud_coop_uv/SetController.h"
#include <unordered_map>



class MergeNode
{
public:
    MergeNode();
    ros::NodeHandle nh;

    void tracking_callback(const geometry_msgs::Twist& cmd_vel);
    void coverage_callback(const geometry_msgs::Twist& cmd_vel);
    void joy_callback(const geometry_msgs::Twist& cmd_vel);

    void merge_cmd_vel(void);

    bool set_controller_callback(tud_coop_uv::SetController::Request& request,
                                 tud_coop_uv::SetController::Response& response);


private:
    ros::Subscriber tracking_sub_;
    ros::Subscriber coverage_sub_;
    ros::Subscriber joy_sub_;

    ros::Publisher cmd_vel_pub_;
    ros::Publisher cmd_vel_stamped_pub_;

    geometry_msgs::Twist cmd_vel_tracking_;
    geometry_msgs::Twist cmd_vel_coverage_;
    geometry_msgs::Twist cmd_vel_joy_;

    ros::ServiceServer set_controller_srv_;

    std::unordered_map<std::string,uint> available_controllers_;
    std::string current_controller_;
};

#endif // MERGENODE_HPP
