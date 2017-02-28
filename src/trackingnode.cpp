#include "tud_coop_uv/trackingnode.hpp"

TrackingNode::TrackingNode() {
  target_updated_sub_ = nh_.subscribe("/define_target/target_updated", 1, &TrackingNode::update_control,
                                     this, ros::TransportHints().tcpNoDelay());

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/tracking/cmd_vel", 1);
  debug_pub_ = nh_.advertise<geometry_msgs::Twist>("/tracking/yaw_error", 1);
  debug_land_ = nh_.advertise<geometry_msgs::Twist>("/tracking/land_sent", 1);
  cmd_vel_marker_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/cmd_vel_marker", 1);

  ardrone_land_pub_ = nh_.advertise<std_msgs::Empty>("/ardrone/land", 1);

  // Dynamic parameter reconfigure
  dynamic_reconfigure::Server<tud_coop_uv::tracking_dynamic_param_configConfig>::CallbackType f;
  f = boost::bind(&TrackingNode::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
  controlled_landing_ = false;
}

void TrackingNode::dynamicReconfigureCb(
    tud_coop_uv::tracking_dynamic_param_configConfig& config, uint32_t level){

  ROS_INFO("Reconfigure Request: %f", config.ref_quadcopter_height);

  ref_quadcopter_height_ = config.ref_quadcopter_height;

  max_linear_velocity_ = config.max_linear_velocity;
  max_yaw_velocity_ = config.max_yaw_velocity;
  max_z_velocity_ = config.max_z_velocity;

  //velocity control

  kp_velxy_ = config.kp_velxy;
  ki_velxy_ = config.ki_velxy;
  kd_velxy_ = config.kd_velxy;


  //Tilting angle control

  kp_tilt_ = config.kp_tilt;
  ki_tilt_ = config.ki_tilt;
  kd_tilt_ = config.kd_tilt;

  kp_z_ = config.kp_z;
  ki_z_ = config.ki_z;
  kd_z_ = config.kd_z;

  kp_yaw_ = config.kp_yaw;
  ki_yaw_ = config.ki_yaw;
  kd_yaw_ = config.kd_yaw;

  controlled_landing_ = config.controlled_landing;
}

void TrackingNode::update_control(const std_msgs::Empty empty_msg){

  tf::Pose target_pose;
  bool valid_pose = get_target_pose(target_pose);
  if (valid_pose) {
    tracking_control(target_pose);
  } else {
    //! HOVER
    set_hover();
  }
}

bool TrackingNode::get_target_pose(tf::Pose& target_pose) {
  // Transformation from target frame to quad_base

  // Clear the target_pose variable
  target_pose.setIdentity();

  tf::StampedTransform base_to_target;
  ros::Time now = ros::Time::now();

  // We look for a transformation betwwen quadcopter_base_link and
  // tracking_target
  try {
    tf_listener_.waitForTransform("/ardrone_base_link", "/tracking_target", now,
                                  ros::Duration(0.50));
    tf_listener_.lookupTransform("/ardrone_base_link", "/tracking_target", now,
                                 base_to_target);
    // ROS_DEBUG("tracking_node | Parent Frame: %s, Child frame: %s",
    // base_to_target.frame_id_.c_str(),
    // base_to_target.child_frame_id_.c_str());
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    // if we dont have a valid transformation return false
    return false;
  }

  // Now we create a new fixed frame transformation: /ardrone_base_link_fixed
  // This will give us an ardrone frame that only changes in yaw
  // in the same position of the /ardrone_base_link frame
  double roll, pitch, yaw;
  tf::StampedTransform stampedTransform_fixed;
  tf::Transform target_to_fixed, base_to_fixed;

  base_to_target.inverse().getBasis().getRPY(roll, pitch, yaw);
  target_to_fixed.setOrigin(base_to_target.inverse().getOrigin());
  target_to_fixed.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw));
  base_to_fixed = base_to_target * target_to_fixed;

  // We publish the new stamped transform to the TF server
  stampedTransform_fixed.child_frame_id_ = "/ardrone_base_link_fixed";
  stampedTransform_fixed.frame_id_ = "/ardrone_base_link";
  stampedTransform_fixed.setData(base_to_fixed);
  stampedTransform_fixed.stamp_ = ros::Time::now();
  tf_broadcaster_.sendTransform(stampedTransform_fixed);

  // Obtain tracking_target pose in ardrone fixed frame
  // this is the final output of the function
  target_pose = target_to_fixed.inverse() * target_pose;

  // We draw a tracking arrow in RVIZ for visual reference
  tf::Vector3 marker_position = target_pose.getOrigin();
  marker_position.setZ(0.0);
  draw_arrow_rviz(marker_position);

  return true;
}

void TrackingNode::tracking_control(tf::Pose& target_pose) {
  // In the frame /ardrone_base_link_fixed
  double velx, vely, velz;
  double tiltx, tilty;
  tf::Point target_point = target_pose.getOrigin();

  // For derivative and integral we need the current time and timestep (dt)
  t = ros::Time::now();
  ros::Duration dt = t - old_t;
  old_t = t;

  //! position controller based on a velocity approach vector
  velxy_control(target_pose, velx, vely);

  //! position controller based on tilting angle
  //tiltxy_control(target_pose, tiltx, tilty, dt);

  //! height controller
  height_control(target_pose, velz, dt);

  //! Yaw controller
  double error_yaw, yaw_ang_speed;
  error_yaw = tf::getYaw(target_pose.getRotation());
  yaw_ang_speed = error_yaw * kp_yaw_;
  // for debugging
  geometry_msgs::Twist debug_msg;
  debug_msg.linear.x = error_yaw;
  debug_pub_.publish(debug_msg);

  // limit max angular speed
  yaw_ang_speed = std::max(std::min(yaw_ang_speed, max_yaw_velocity_), -max_yaw_velocity_);


  // ROS_DEBUG("current_height:  %f | expected_height:  %f | height_error:  %f |
  // ", current_height, expected_height, height_error);
  // ROS_DEBUG("velz:  %f", velz);


  geometry_msgs::Twist cmd_vel_out;
  cmd_vel_out.linear.x = velx;
  cmd_vel_out.linear.y = vely;
  cmd_vel_out.linear.z = velz;  
  cmd_vel_out.angular.z = yaw_ang_speed;  
  // We dont want to hover.
  cmd_vel_out.angular.x = 1;
  cmd_vel_out.angular.y = 1;

  cmd_vel_pub_.publish(cmd_vel_out);
  // draw_arrow_rviz(tracking_point);


}


//Velocity based tracking control (to be used with ardrone_velocity)
void TrackingNode::velxy_control(tf::Pose& target_pose, double& velx, double& vely){
  //! position controller based on a velocity approach vector

  tf::Point target_point = target_pose.getOrigin();
  velx = target_point.x() * kp_velxy_;
  vely = target_point.y() * kp_velxy_;

  // for debugging
//  geometry_msgs::Twist debug_msg;
//  debug_msg.linear.x = target_point.x();
//  debug_msg.linear.y = target_point.y();
//  debug_pub_.publish(debug_msg);


  // limit the module of speed
  double norm = sqrt(velx * velx + vely * vely);
  if (norm > max_linear_velocity_) {
    velx = (velx / norm) * max_linear_velocity_;
    vely = (vely / norm) * max_linear_velocity_;
  }

}

//Direct tilt control (without ardrone_velocity, commands direct to ardrone autonomy)
void TrackingNode::tiltxy_control(tf::Pose& target_pose, double& tiltx, double& tilty, ros::Duration dt){
  double error_x, error_y;
  double p_term_x, p_term_y;
  double d_term_x, d_term_y;

  tf::Point target_point = target_pose.getOrigin();

  // Position error
  error_x = target_point.x();
  error_y = target_point.y();




  // The proportional term is directly the error
  p_term_x = error_x;
  p_term_y = error_y;



  // Derivative term



  error_x_filtered_ = derivative_filter_x.filter(error_x);
  error_y_filtered_ = derivative_filter_y.filter(error_y);

  d_term_x = (error_x_filtered_ - last_error_x_)/(1.0f/25.0);/// dt.toSec();
  d_term_y = (error_y_filtered_ - last_error_y_)/(1.0f/25.0);/// dt.toSec();




  // for debugging
//  geometry_msgs::Twist debug_msg;
//  debug_msg.linear.x = error_x;
//  debug_msg.linear.y = error_y;
//  debug_msg.linear.z = error_x_filtered_;
//  debug_pub_.publish(debug_msg);


  last_error_x_ = error_x;
  last_error_y_ = error_y;


  //! Taken from tum_autonomy package.
  //! This calculates and limits the integral term
  // m_i_term is a member of the class
  i_term_increase(i_term_x_, error_x * dt.toSec(), 0.1f / ki_tilt_+(1e-10));
  i_term_increase(i_term_y_, error_y * dt.toSec(), 0.1f / ki_tilt_+(1e-10));

  // kill integral term when first crossing target
  // that is, thargetNew is set, it was set at least 100ms ago, and err changed sign.

  // If error changed sign
  if(error_x*last_error_x_){
    i_term_x_ = 0;
  }
  if(error_y*last_error_y_){
    i_term_y_ = 0;
  }


//  for(int i=0;i<4;i++)
//    if(targetNew[i] > 0.5 && getMS()/1000.0 - targetSetAtClock > 0.1 && last_err[i] * new_err[i] < 0)
//    {
//      i_term[i] = 0; targetNew[i] = 0;
//    }



  // Control command (PID)
  tiltx = kp_tilt_*p_term_x + ki_tilt_*i_term_x_ + kd_tilt_*d_term_x;
  tilty = kp_tilt_*p_term_y + ki_tilt_*i_term_y_ + kd_tilt_*d_term_y;


  // limit the max tilting angle
  double max_tilt_angle = 1.0;
  tiltx = std::max(std::min(tiltx, max_tilt_angle), -max_tilt_angle);
  tilty = std::max(std::min(tilty, max_tilt_angle), -max_tilt_angle);



}



void TrackingNode::height_control(tf::Pose& target_pose, double& velz, ros::Duration dt){
  double error_z, current_height;
  double p_term_z, d_term_z;
  velz = 0.0;
  tf::Point target_point = target_pose.getOrigin();
  current_height = -target_point.z(); //The marker position

  if(controlled_landing_ == true){
    velz = -0.15;
    velz = std::max(std::min(velz, max_z_velocity_), -max_z_velocity_);
    if(current_height < 0.40){
      velz = -0.1;

      if(current_height <0.30){
        velz = -0.07;
      }
      if(current_height <0.25){
        velz = 0.0;
        // check for forced landing
        double dx, dy, error;
        dx = target_point.x();
        dy = target_point.y();

        error = sqrt(dx*dx + dy*dy);
        if (sqrt(dx*dx) < 0.05 && sqrt(dy*dy) <0.05){

          ROS_INFO("Sending forced land command, height: %f", current_height);
          std_msgs::Empty empty;
          ardrone_land_pub_.publish(empty);
          // for debugging
          geometry_msgs::Twist debug_msg;
          debug_msg.linear.x = 5.0;
          debug_land_.publish(debug_msg);
        }
      }


    }
    return;
  }

  //We force bnlind landing if it is too close to the marker
//  if(current_height < 0.5){
//      std_msgs::Empty empty;
//      ardrone_land_pub_.publish(empty);
//      ROS_INFO("Alert, minimum marker to quad distance, sending blind landing command");
//      return;
//  }

  error_z = ref_quadcopter_height_ - current_height;

  // The proportional term is directly the error
  p_term_z = error_z;


  // Derivative term
  d_term_z = (error_z - last_error_z_);// / dt.toSec();


  last_error_z_ = error_z;

  //! Taken from tum_autonomy package.
  //! This calculates and limits the integral term
  i_term_increase(i_term_z_, error_z * dt.toSec(), 0.2f / ki_z_);

  // kill integral term when first crossing target
  // that is, thargetNew is set, it was set at least 100ms ago, and err changed sign.
  // If error changed sign
  if(error_z*last_error_z_){
    i_term_z_ = 0;
  }


  // Control command (PID)
  velz = kp_z_*p_term_z + ki_z_*i_term_z_ + kd_z_*d_term_z;


  // limit max vertical speed
  velz = std::max(std::min(velz, max_z_velocity_), -max_z_velocity_);

}

void TrackingNode::draw_arrow_rviz(tf::Vector3& endpoint) {
  // Publishes a Marker (ARROW) in RVIZ
  // from the center of the quadcopter frame to the center of the AruCo Board
  // frame
  // We publish a ARROW for RVIZ representing the tracking
  // x, y must be in quadcopter coordinate frame
  visualization_msgs::Marker cmd_vel_marker;
  cmd_vel_marker.header.frame_id = "/ardrone_base_link_fixed";
  cmd_vel_marker.header.stamp = ros::Time::now();
  cmd_vel_marker.ns = "tud_coop_uv_tracking";
  cmd_vel_marker.id = 0;
  cmd_vel_marker.type = visualization_msgs::Marker::ARROW;
  cmd_vel_marker.action = visualization_msgs::Marker::ADD;
  cmd_vel_marker.scale.x = 0.01;
  cmd_vel_marker.scale.y = 0.04;
  cmd_vel_marker.scale.z = 0.01;
  cmd_vel_marker.color.a = 1.0;  // Don't forget to set the alpha!
  cmd_vel_marker.color.r = 0.0;
  cmd_vel_marker.color.g = 1.0;
  cmd_vel_marker.color.b = 0.0;

  // We define the start and end points of the ARROW
  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = 0.0;
  cmd_vel_marker.points.push_back(p);
  p.x = endpoint.x();
  p.y = endpoint.y();
  p.z = endpoint.z();
  cmd_vel_marker.points.push_back(p);
  cmd_vel_marker_pub_.publish(cmd_vel_marker);
}


void TrackingNode::set_hover(void) {
  geometry_msgs::Twist cmd_vel_out;
  cmd_vel_out.linear.x = 0;
  cmd_vel_out.linear.y = 0;
  cmd_vel_out.linear.z = 0;
  cmd_vel_out.angular.x = 0;
  cmd_vel_out.angular.y = 0;
  cmd_vel_out.angular.z = 0;
  cmd_vel_pub_.publish(cmd_vel_out);
}



void TrackingNode::i_term_increase(double& i_term, double new_err, double cap)
{
  if(new_err < 0 && i_term > 0)
    i_term = std::max(0.0, i_term + 2.5 * new_err);
  else if(new_err > 0 && i_term < 0)
    i_term = std::min(0.0, i_term + 2.5 * new_err);
  else
    i_term += new_err;

  if(i_term > cap) i_term =  cap;
  if(i_term < -cap) i_term =  -cap;
}


