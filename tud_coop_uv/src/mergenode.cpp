#include "tud_coop_uv/mergenode.hpp"

MergeNode::MergeNode()
{
  ros::NodeHandle params("~");
  std::string output_topic;

  params.param<std::string>("output_topic", output_topic, "/merge/cmd_vel");

    available_controllers_ = {{"joy",0},{"tracking",1},{"joint",2}};
    tracking_sub_ = nh.subscribe ("/tracking/cmd_vel", 1, &MergeNode::tracking_callback, this);
    coverage_sub_ = nh.subscribe ("/coverage/cmd_vel", 1, &MergeNode::coverage_callback, this);
    joy_sub_ = nh.subscribe ("/joy/cmd_vel", 1, &MergeNode::joy_callback, this);

    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(output_topic,1);
    cmd_vel_stamped_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/merge/cmd_vel_stamped",1);

    set_controller_srv_ = nh.advertiseService("tud_coop_uv/set_controller", &MergeNode::set_controller_callback,this);
    current_controller_ = "joy";
}

void MergeNode::merge_cmd_vel(void){
    geometry_msgs::Twist cmd_vel_out;
    geometry_msgs::TwistStamped cmd_vel_out_stamped;
    if (current_controller_ == "joy"){
       cmd_vel_out = cmd_vel_joy_;
       cmd_vel_out.linear.x *= 0.6;
       cmd_vel_out.linear.y *= 0.6;
    }
    else if (current_controller_ == "tracking"){
        cmd_vel_out = cmd_vel_tracking_;
    }
    else if (current_controller_ == "joint"){

        cmd_vel_out.linear.x = cmd_vel_tracking_.linear.x +
                               cmd_vel_coverage_.linear.x +
                               cmd_vel_joy_.linear.x*0.6;
        cmd_vel_out.linear.y = cmd_vel_tracking_.linear.y +
                cmd_vel_coverage_.linear.y +
                cmd_vel_joy_.linear.y*0.6;
        cmd_vel_out.linear.z = cmd_vel_tracking_.linear.z +
                cmd_vel_coverage_.linear.z +
                cmd_vel_joy_.linear.z;

        if (std::abs(cmd_vel_joy_.angular.z) > 0.01){
          cmd_vel_out.angular.z = cmd_vel_joy_.angular.z;
        }
        else{
          cmd_vel_out.angular.z = cmd_vel_tracking_.angular.z +
                  cmd_vel_coverage_.angular.z;
        }
        //hack for manual angular control during joint control
        //cmd_vel_out.angular.z = m_cmd_vel_joy.angular.z;
    }
    cmd_vel_out_stamped.twist = cmd_vel_out;
    cmd_vel_out_stamped.header.stamp = ros::Time::now();

    cmd_vel_pub_.publish(cmd_vel_out);
    cmd_vel_stamped_pub_.publish(cmd_vel_out_stamped);

}

void MergeNode::tracking_callback(const geometry_msgs::Twist& cmd_vel){
  if(current_controller_ == "tracking" || current_controller_ == "joint"){
    //save current tracking command
    cmd_vel_tracking_ = cmd_vel;
    merge_cmd_vel();
  }
}

void MergeNode::coverage_callback(const geometry_msgs::Twist& cmd_vel){
  if(current_controller_ == "coverage"){
    cmd_vel_coverage_ = cmd_vel;
    merge_cmd_vel();
  }
}

void MergeNode::joy_callback(const geometry_msgs::Twist& cmd_vel){
  if(current_controller_ == "joy" || current_controller_ == "joint"){
    cmd_vel_joy_ = cmd_vel;
    merge_cmd_vel();
  }
}

bool MergeNode::set_controller_callback(tud_coop_uv::SetController::Request& request,
                             tud_coop_uv::SetController::Response& response){
    if (request.controller == available_controllers_["joy"]){
        ROS_WARN("Joy controller");
        current_controller_ = "joy";
    }
    else if (request.controller == available_controllers_["tracking"]){
        ROS_WARN("Tracking autonomous controller");
        current_controller_ = "tracking";
    }
    else if (request.controller == available_controllers_["joint"]){
        ROS_WARN("Joint controller");
        current_controller_ = "joint";
    }
    else{
        ROS_ERROR("ERROR wrong controller value");
        current_controller_ = "joy";
        response.result = false;
        return false;
    }
    response.result = true;
    return true;
}
