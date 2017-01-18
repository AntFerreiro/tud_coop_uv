#include "tud_coop_uv/trackingnode.hpp"

TrackingNode::TrackingNode() {
  // ROS parameters definition
  nh_.param<double>("ref_quadcopter_height", ref_quadcopter_height_, 1.1); // in meters

  quad_odom_sub_ =
      nh_.subscribe("/ardrone/odometry", 1, &TrackingNode::quad_OdomCallback,
                   this, ros::TransportHints().tcpNoDelay());

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/tracking/cmd_vel", 1);
  debug_pub_ = nh_.advertise<std_msgs::Float64>("/debug", 1);
  cmd_vel_marker_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/cmd_vel_marker", 1);
}

void TrackingNode::quad_OdomCallback(const nav_msgs::Odometry& odo_msg) {
  // With each incoming odometry message we try to find a
  // transformation to the target and then do the tracking
  odo_msg_ = odo_msg;
  tf::Pose target_pose;
  bool valid_pose;
  valid_pose = get_target_pose(target_pose);
  if (valid_pose) {
    tracking_control(target_pose);
  } else {
    //! HOVER
    set_hover();
  }

  // for debugging
  // std_msgs::Float64 debug_msg;
  // debug_msg.data = m_filtered_vel_x;
  // m_debug_pub.publish(debug_msg);
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

void TrackingNode::tracking_control(tf::Pose target_pose) {
  // In the frame /ardrone_base_link_fixed
  double velx, vely, velz, px, py, pz, pyaw;
  double height_error, current_height;
  tf::Point target_point = target_pose.getOrigin();
  // target_point.setZ(0.0);

  //! position controller
  px = py = 0.75;
  velx = target_point.x() * px;
  vely = target_point.y() * py;
  // limit maximum module of speed
  double norm = sqrt(velx * velx + vely * vely);
  //!Completly arbitrary number (change with ros_param)
  if (norm > 0.4) {
    velx = (velx / norm) * 0.4;
    vely = (vely / norm) * 0.4;
  }

  //! Yaw controller
  double error_yaw, yaw_ang_speed;
  error_yaw = tf::getYaw(target_pose.getRotation());
  pyaw = 0.2;
  yaw_ang_speed = error_yaw * pyaw;

  // for debugging
  std_msgs::Float64 debug_msg;
  debug_msg.data = yaw_ang_speed;
  debug_pub_.publish(debug_msg);
  // limit max angular speed
  yaw_ang_speed = std::max(std::min(yaw_ang_speed, 0.2), -0.2);

  //! height controller
  pz = 0.3;
  current_height = -target_point.z();
  height_error = ref_quadcopter_height_ - current_height;
  velz = height_error * pz;
  // limit max vertical speed
  velz = std::max(std::min(height_error, 0.2), -0.2);

  // ROS_INFO("----------------------------");
  // ROS_INFO("current_height:  %f | expected_height:  %f | height_error:  %f |
  // ", current_height, expected_height, height_error);
  // ROS_INFO("velz:  %f", velz);
  // ROS_INFO("----------------------------");

  geometry_msgs::Twist cmd_vel_out;
  cmd_vel_out.linear.x = velx;
  cmd_vel_out.linear.y = vely;
  cmd_vel_out.linear.z = velz;
  cmd_vel_out.angular.z = yaw_ang_speed;
  cmd_vel_pub_.publish(cmd_vel_out);
  // draw_arrow_rviz(tracking_point);
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

void TrackingNode::arsys_marker_pose_callback(
    const geometry_msgs::PoseStamped& marker_pose_msg) {
  //  tf::StampedTransform transform_ardrone_board;  // boardFiltered ->
  //                                                 // ardrone_base_bottomcam
  //                                                 ->
  //                                                 // ardrone_base_link
  //  try {
  //    tf_listener_.lookupTransform("/boardFiltered", "/ardrone_base_link",
  //                                  ros::Time(0), transform_ardrone_board);

  //  } catch (tf::TransformException& ex) {
  //    ROS_ERROR("%s", ex.what());
  //  }

  //  // Now we create a new fixed frame transformation:
  //  /ardrone_base_link_fixed
  //  // This will give us an ardrone frame that only changes in yaw
  //  // in the same position of the /ardrone_base_link frame
  //  double roll, pitch, yaw;
  //  tf::StampedTransform stampedTransform_fixed;
  //  tf::Transform transform_fixed;
  //  transform_ardrone_board.getBasis().getRPY(roll, pitch, yaw);

  //  transform_fixed.setOrigin(transform_ardrone_board.getOrigin());
  //  transform_fixed.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw));

  //  // We publish the new stamped transform for RVIZ
  //  stampedTransform_fixed.child_frame_id_ = "/ardrone_base_link_fixed";
  //  stampedTransform_fixed.frame_id_ = "/boardFiltered";
  //  stampedTransform_fixed.setData(transform_fixed);
  //  stampedTransform_fixed.stamp_ = ros::Time::now();
  //  m_tf_broadcaster.sendTransform(stampedTransform_fixed);

  //  // Obtain marker position in ardrone fixed frame
  //  tf::Vector3 marker_position(0, 0, 0);
  //  marker_position = stampedTransform_fixed.inverse() * marker_position;

  //  // We draw a tracking arrow in RVIZ for visual reference
  //  draw_arrow_rviz(marker_position);

  //  // Now we do the tracking
  //  tracking_control(marker_position);

  //  //! TODO implement orientation PID controller using yaw variable
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
