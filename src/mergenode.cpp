#include "tud_coop_uv/mergenode.hpp"

MergeNode::MergeNode()
{
    m_tracking_sub = nh.subscribe ("/tracking/cmd_vel", 1, &MergeNode::tracking_callback, this);
    m_coverage_sub = nh.subscribe ("/coverage/cmd_vel", 1, &MergeNode::coverage_callback, this);
    m_joy_sub = nh.subscribe ("/joy/cmd_vel", 1, &MergeNode::joy_callback, this);

    m_set_controller_srv = nh.advertiseService("tud_coop_uv/set_controller", &MergeNode::set_controller_callback,this);
}

void MergeNode::tracking_callback(const geometry_msgs::Twist& cmd_vel){
    //save current tracking command
    m_cmd_vel_tracking = cmd_vel;
    //if coverage received
}

void MergeNode::coverage_callback(const geometry_msgs::Twist& cmd_vel){
    m_cmd_vel_coverage = cmd_vel;

}

void MergeNode::joy_callback(const geometry_msgs::Twist& cmd_vel){
    m_cmd_vel_joy = cmd_vel;
}

bool MergeNode::set_controller_callback(tud_coop_uv::SetController::Request& request,
                             tud_coop_uv::SetController::Response& response){
    //request.controller
    request.controller;
    return true;
}
