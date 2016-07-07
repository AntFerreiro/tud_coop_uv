#include "tud_coop_uv/trackingnode.hpp"

TrackingNode::TrackingNode() {
  //! TODO new parameter
  //! set_target:
  //!   values: marker_frame, middle, centered on one with view of the others.

  m_quad_vel_sub =
      nh.subscribe("/ardrone/odometry", 1, &TrackingNode::quad_OdomCallback,
                   this, ros::TransportHints().tcpNoDelay());

  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/tracking/cmd_vel", 1);
  debug_pub_ = nh.advertise<std_msgs::Float64>("/pid/debug", 1);
  cmd_vel_marker_pub_ =
      nh.advertise<visualization_msgs::Marker>("/cmd_vel_marker", 1);
}

void TrackingNode::quad_OdomCallback(const nav_msgs::Odometry& odo_msg) {
  // With each incoming odometry message we try to find a
  // transformation to the target and then do the tracking
  odo_msg_ = odo_msg;
  tf::Point target_point = get_target_point();
  tracking_control(target_point);
  //for debugging
  //std_msgs::Float64 debug_msg;
  // debug_msg.data = m_filtered_vel_x;
  // m_debug_pub.publish(debug_msg);
}

tf::Point TrackingNode::get_target_point(void){
  //Transformation from target frame to quad_base
  tf::StampedTransform base_to_target;
  ros::Time now = ros::Time::now();
  try {
    tf_listener_.waitForTransform("/ardrone_base_link", "/tracking_target",  now, ros::Duration(0.50));
    tf_listener_.lookupTransform("/ardrone_base_link", "/tracking_target", now, base_to_target);
    ROS_INFO("Parent Frame: %s, Child frame: %s", base_to_target.frame_id_.c_str(), base_to_target.child_frame_id_.c_str());
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return base_to_target.getOrigin();
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


    base_to_fixed = base_to_target*target_to_fixed;

    // We publish the new stamped transform for RVIZ
    stampedTransform_fixed.child_frame_id_ = "/ardrone_base_link_fixed";
    stampedTransform_fixed.frame_id_ = "/ardrone_base_link";
    stampedTransform_fixed.setData(base_to_fixed);
    stampedTransform_fixed.stamp_ = ros::Time::now();
    m_tf_broadcaster.sendTransform(stampedTransform_fixed);

    // Obtain marker position in ardrone fixed frame
    tf::Pose flight_target;
    tf::Vector3 marker_position(0, 0, 0);

    //! TODO implement pose tracking
    flight_target = target_to_fixed.inverse() *flight_target;
    marker_position = flight_target.getOrigin();
    marker_position.setZ(0.0);

    // We draw a tracking arrow in RVIZ for visual reference
    draw_arrow_rviz(marker_position);
  return marker_position;
}

void TrackingNode::tracking_control(tf::Vector3& tracking_point) {
  // In the frame /ardrone_base_link_fixed
  double velx;
  double vely;

  velx = tracking_point.x() * 0.75;
  vely = tracking_point.y() * 0.75;

  double norm = sqrt(velx * velx + vely * vely);
  //!Completly arbitrary number (change with ros_param)
  if (norm > 0.4) {
    velx = (velx / norm) * 0.4;
    vely = (vely / norm) * 0.4;
  }

  geometry_msgs::Twist cmd_vel_out;
  cmd_vel_out.linear.x = velx;
  cmd_vel_out.linear.y = vely;
  cmd_vel_pub_.publish(cmd_vel_out);
  //draw_arrow_rviz(tracking_point);
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
//                                                 // ardrone_base_bottomcam ->
//                                                 // ardrone_base_link
//  try {
//    tf_listener_.lookupTransform("/boardFiltered", "/ardrone_base_link",
//                                  ros::Time(0), transform_ardrone_board);

//  } catch (tf::TransformException& ex) {
//    ROS_ERROR("%s", ex.what());
//  }

//  // Now we create a new fixed frame transformation: /ardrone_base_link_fixed
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
