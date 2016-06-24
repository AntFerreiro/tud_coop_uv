#include "tud_coop_uv/definetargetnode.hpp"

DefineTargetNode::DefineTargetNode() {
  // if only one ARSYS marker
  /// check conditions for more markers (maybe dynamic reconfigure)


  std::string marker_tf_topic, marker_pose_topic;
  // ugv stands for unmanned ground vehicle
  // uav stands for unmanned aerial vehicle

  nh_.param<std::string>("marker_tf_topic", marker_tf_topic, "/arsys_single_board/transform");

  nh_.param<std::string>("marker_pose_topic", marker_pose_topic, "/ar_single_board/pose");

  /// TODO catch if number of ugv small than 1 or greater than 2 (not implemented)
  nh_.param<int>("n_ugv", n_ugv_, 1);

  std::string ugv_frame, ugv_marker_frame;
  for (int i=0; i<n_ugv_; i++){
    nh_.param<std::string>("ugv_frame_"+i, ugv_frame);
    ugv_frame_.push_back(ugv_frame);
    ugv_marker_frame_ = ugv_frame + "/top_marker";
    ugv_marker_frame_.push_back(ugv_marker_frame);
  }

  nh_.param<std::string>("uav_base_link_", uav_base_link_, "/ardrone_base_link");

  arsys_transform_sub_ =
      nh_.subscribe(marker_tf_topic, 1,
                   &DefineTargetNode::arsys_transform_callback, this);
  arsys_pose_sub_ =
      nh_.subscribe(marker_pose_topic, 1,
                   &DefineTargetNode::arsys_marker_pose_callback, this);
}

/// Only as reference to see what to send to tracking node
// void DefineTargetNode::tracking_control(tf::Vector3& tracking_point){
//    // In the frame /ardrone_base_link_fixed
//    double velx;
//    double vely;

//    velx = tracking_point.x()*0.75;
//    vely = tracking_point.y()*0.75;

//    double norm = sqrt(velx*velx + vely*vely);
//    //!Completly arbitrary number (change with ros_param)
//    if (norm > 0.4){
//        velx = (velx/norm)*0.4;
//        vely = (vely/norm)*0.4;
//    }

//    geometry_msgs::Twist cmd_vel_out;
//    cmd_vel_out.linear.x = velx;
//    cmd_vel_out.linear.y = vely;
//    m_cmd_vel_pub.publish(cmd_vel_out);
//}

/// Only for one marker
void DefineTargetNode::arsys_marker_pose_callback(
    const geometry_msgs::PoseStamped& marker_pose_msg) {
//  tf::StampedTransform transform_ardrone_board;  // boardFiltered -> // ardrone_base_bottomcam ->// ardrone_base_link
//  try {
//    m_tf_listener.lookupTransform("/boardFiltered", "/ardrone_base_link",
//                                  ros::Time(0), transform_ardrone_board);

//  } catch (tf::TransformException& ex) {
//    ROS_ERROR("%s", ex.what());
//  }

  // Now we create a new fixed frame transformation: /ardrone_base_link_fixed
  // This will give us an ardrone frame that only changes in yaw
  // in the same position of the /ardrone_base_link frame
  double roll, pitch, yaw;
  tf::StampedTransform stampedTransform_fixed;
  tf::Transform transform_fixed;
  transform_ardrone_board.getBasis().getRPY(roll, pitch, yaw);

  transform_fixed.setOrigin(transform_ardrone_board.getOrigin());
  transform_fixed.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw));

  // We publish the new stamped transform for RVIZ
  stampedTransform_fixed.child_frame_id_ = "/ardrone_base_link_fixed";
  stampedTransform_fixed.frame_id_ = "/boardFiltered";
  stampedTransform_fixed.setData(transform_fixed);
  stampedTransform_fixed.stamp_ = ros::Time::now();
  m_tf_broadcaster.sendTransform(stampedTransform_fixed);

  // Obtain marker position in ardrone fixed frame
  tf::Vector3 marker_position(0, 0, 0);
  marker_position = stampedTransform_fixed.inverse() * marker_position;

  // We draw a tracking arrow in RVIZ for visual reference
  draw_arrow_rviz(marker_position);

  // Now we do the tracking
  /// Here we publish the stamped Pose to tracking node
  // tracking_control(marker_position);

  //! TODO implement orientation PID controller using yaw variable
}

/// Understand this funciton, maybe I need several for other markers.
void DefineTargetNode::arsys_transform_callback(
    const geometry_msgs::TransformStamped& transformMsg) {
  //! This message comes from the ar_sys package.
  //! It has the transform of a board_frame to the camera_frame
  //! There is one message for each board

  // First we convert the message to a StampedTransform object
  tf::StampedTransform stampedTransform_in;
  tf::transformStampedMsgToTF(transformMsg, stampedTransform_in);

  m_transform = tf_filter(m_transform, stampedTransform_in);

//  double x = m_transform.getOrigin().x();
//  double y = m_transform.getOrigin().y();
//  ROS_INFO("Posicion Patron: (%f,%f)",x,y);

  // Now we create a Stamped transform from the filtered transform
  // with a filtered sufix
  tf::StampedTransform stampedTransform_out(
      m_transform, transformMsg.header.stamp, transformMsg.header.frame_id,
      transformMsg.child_frame_id+"_filtered");

  // We broadcast the transformations to the TF Server
  // Original transformation without filter
  m_tf_broadcaster.sendTransform(stampedTransform_in);
  // Filtered transformation
  m_tf_broadcaster.sendTransform(stampedTransform_out);


}

void DefineTargetNode::calculate_target_pose( void ){

  //Check if we received all the marker board transformations for a particular time step
  //! TODO


  // Now we want to obtain a transformation from the Quadcopter frame to the Marker Frame
  // /board_filtered -> /camera -> /quadcopter_base_link
  tf::StampedTransform marker_to_base_tf[n_ugv_];

  try {
    m_tf_listener.lookupTransform(ugv_marker_frame_[n_ugv_], uav_base_link_,
                                  ros::Time(0), marker_to_base_tf[n_ugv_]);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
  }



  // Now we create a new fixed frame transformation: /base_link_fixed
  // This will give us a quadcopter frame that only changes in yaw
  // in the same position of the /ardrone_base_link frame
  //! TODO(racuna): We are assuming that the ground is flat, so any kind
  //! of inclination in the relative pose is because of the inclination of
  //! the quadcopter. We will have to include the quadcopter IMU information
  //! if we want this to work in inclined surfaces

  double roll, pitch, yaw;
  tf::StampedTransform stampedTransform_fixed;
  tf::Transform transform_fixed;
  marker_to_base_tf[0].getBasis().getRPY(roll, pitch, yaw);

  transform_fixed.setOrigin(transform_ardrone_board.getOrigin());
  transform_fixed.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw));


  tf::StampedTransform target_position;



}

tf::Transform& DefineTargetNode::tf_filter(tf::Transform& dst,
                                                     const tf::Transform& src) {
  //! Taken from ar_sys system_viewer
  //! A digital filter for the detection.
  // It works but we found some singularities in the borders with some
  // inclinations.
  // We changed the digital_filter_change_rate variable from 0.5 to 0.1 and it
  // works better

  double digital_filter_change_rate = 0.1;

  tf::Vector3 posOld = dst.getOrigin();
  tf::Vector3 posNew = src.getOrigin();
  tf::Vector3 pos((1 - digital_filter_change_rate) * posOld.x() +
                      digital_filter_change_rate * posNew.x(),
                  (1 - digital_filter_change_rate) * posOld.y() +
                      digital_filter_change_rate * posNew.y(),
                  (1 - digital_filter_change_rate) * posOld.z() +
                      digital_filter_change_rate * posNew.z());
  dst.setOrigin(pos);

  // I modified this in order to avoid singularities in angles
  tf::Quaternion ornOld = dst.getRotation();
  tf::Quaternion ornNew = src.getRotation();
  tf::Quaternion orn;

  // Interpolation between two quaternions.
  orn = ornOld.slerp(ornNew, digital_filter_change_rate);
  dst.setRotation(orn);

  return dst;
}

void DefineTargetNode::draw_arrow_rviz(tf::Vector3& endpoint) {
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
  p.z = 0.0;
  cmd_vel_marker.points.push_back(p);
  cmd_vel_marker_pub_.publish(cmd_vel_marker);
}
