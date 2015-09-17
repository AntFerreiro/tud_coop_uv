#include "tud_coop_uv/mergenode.hpp"

MergeNode::MergeNode()
{
    m_available_controllers = {{"joy",0},{"tracking",1},{"joint",2}};
    m_tracking_sub = nh.subscribe ("/tracking/cmd_vel", 1, &MergeNode::tracking_callback, this);
    m_coverage_sub = nh.subscribe ("/coverage/cmd_vel", 1, &MergeNode::coverage_callback, this);
    m_joy_sub = nh.subscribe ("/joy/cmd_vel", 1, &MergeNode::joy_callback, this);

    m_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_pid",1);

    m_set_controller_srv = nh.advertiseService("tud_coop_uv/set_controller", &MergeNode::set_controller_callback,this);
}

void MergeNode::merge_cmd_vel(void){
    geometry_msgs::Twist cmd_vel_out;
    if (m_current_controller == "joy"){
       cmd_vel_out = m_cmd_vel_joy;
    }
    else if (m_current_controller == "tracking"){
        cmd_vel_out = m_cmd_vel_tracking;
    }
    else if (m_current_controller == "joint"){

        cmd_vel_out.linear.x = m_cmd_vel_tracking.linear.x +
                               m_cmd_vel_coverage.linear.x +
                               m_cmd_vel_joy.linear.x;
        cmd_vel_out.linear.y = m_cmd_vel_tracking.linear.y +
                m_cmd_vel_coverage.linear.y +
                m_cmd_vel_joy.linear.y;
        cmd_vel_out.linear.z = m_cmd_vel_tracking.linear.z +
                m_cmd_vel_coverage.linear.z +
                m_cmd_vel_joy.linear.z;

        //! I have to decide if orientation is integrated in this message.
    }
     m_cmd_vel_pub.publish(cmd_vel_out);
}

void MergeNode::tracking_callback(const geometry_msgs::Twist& cmd_vel){
    //save current tracking command
    m_cmd_vel_tracking = cmd_vel;
    merge_cmd_vel();
}

void MergeNode::coverage_callback(const geometry_msgs::Twist& cmd_vel){
    m_cmd_vel_coverage = cmd_vel;
    merge_cmd_vel();
}

void MergeNode::joy_callback(const geometry_msgs::Twist& cmd_vel){
    m_cmd_vel_joy = cmd_vel;
    merge_cmd_vel();
}

bool MergeNode::set_controller_callback(tud_coop_uv::SetController::Request& request,
                             tud_coop_uv::SetController::Response& response){
    if (request.controller == m_available_controllers["joy"]){
        ROS_INFO("Joy controller");
        m_current_controller = "joy";
    }
    else if (request.controller == m_available_controllers["tracking"]){
        ROS_INFO("Tracking autonomous controller");
        m_current_controller = "tracking";
    }
    else if (request.controller == m_available_controllers["joint"]){
        ROS_INFO("tracking and coverage autonomous controller");
        m_current_controller = "joint";
    }
    else{
        ROS_ERROR("ERROR wrong controller value");
        m_current_controller = "joy";
        response.result = false;
        return false;
    }
    response.result = true;
    return true;
}
