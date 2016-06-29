#include "tud_coop_uv/definetargetnode.hpp"

DefineTargetNode::DefineTargetNode() : number_of_received_markers_(0) {
  // if only one ARSYS marker
  time_last_tf_message_ = ros::Time::now();
  /// check conditions for more markers (maybe dynamic reconfigure)
  std::string marker_tf_topic, marker_pose_topic;
  std::string ugv_frame_c3po, ugv_frame_r2d2;
  // ugv stands for unmanned ground vehicle
  // uav stands for unmanned aerial vehicle
  nh_.param<std::string>("marker_tf_topic", marker_tf_topic,
                         "/ar_multi_boards/transform");
  nh_.param<std::string>("marker_pose_topic", marker_pose_topic,
                         "/ar_multi_boards/pose");
  nh_.param<std::string>("uav_base_link_", uav_base_link_,
                         "/ardrone_base_link");

  //! TODO(racuna) fix filtering
  nh_.param<bool>("filter_tf", filter_tf_, false);

  /// TODO(racuna) catch if number of ugv small than 1 or greater than 2 (not
  /// implemented)
  nh_.param<int>("n_ugv", n_ugv_, 2);

  nh_.param<std::string>("ugv_frame_c3po", ugv_frame_c3po, "/c3po");
  nh_.param<std::string>("ugv_frame_r2d2", ugv_frame_r2d2, "/r2d2");
  ugv_frames_.push_back(ugv_frame_c3po);
  ugv_marker_frames_.push_back(ugv_frame_c3po + "/top_marker");
  ugv_frames_.push_back(ugv_frame_r2d2);
  ugv_marker_frames_.push_back(ugv_frame_r2d2 + "/top_marker");

  //If I have several markers this could be a for loop.
  check_received_markers_[ugv_marker_frames_[0]] = false;
  check_received_markers_[ugv_marker_frames_[1]] = false;

  arsys_transform_sub_ = nh_.subscribe(
      marker_tf_topic, 1, &DefineTargetNode::arsys_transform_callback, this);
  arsys_pose_sub_ =
      nh_.subscribe(marker_pose_topic, 1,
                    &DefineTargetNode::arsys_marker_pose_callback, this);
}

void DefineTargetNode::arsys_marker_pose_callback(
    const geometry_msgs::PoseStamped& marker_pose_msg) {
  //  tf::StampedTransform transform_ardrone_board;  // boardFiltered -> //
  //  ardrone_base_bottomcam ->// ardrone_base_link
  //  try {
  //    m_tf_listener.lookupTransform("/boardFiltered", "/ardrone_base_link",
  //                                  ros::Time(0), transform_ardrone_board);

  //  } catch (tf::TransformException& ex) {
  //    ROS_ERROR("%s", ex.what());
  //  }

  // Now we create a new fixed frame transformation: /ardrone_base_link_fixed
  // This will give us an ardrone frame that only changes in yaw
  // in the same position of the /ardrone_base_link frame
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

  // Now we do the tracking
  /// Here we publish the stamped Pose to tracking node
  // tracking_control(marker_position);

  //! TODO implement orientation PID controller using yaw variable
}

/// Understand this function, maybe I need several for other markers.
void DefineTargetNode::arsys_transform_callback(
    const geometry_msgs::TransformStamped& transformMsg) {
  //! This message comes from the ar_sys package.
  //! It has the transform of a board_frame to the camera_frame
  //! There is one message for each board

  // First we convert the message to a StampedTransform object
  tf::StampedTransform stampedTransform_in;
  tf::transformStampedMsgToTF(transformMsg, stampedTransform_in);

  //  double x = m_transform.getOrigin().x();
  //  double y = m_transform.getOrigin().y();
  //  ROS_INFO("Posicion marker " + transformMsg.header.frame_id + ":
  //  (%f,%f)",x,y);
  ROS_INFO("frame: %s, sec: %i, nsec: %i", transformMsg.child_frame_id.c_str(),
           transformMsg.header.stamp.sec, transformMsg.header.stamp.nsec);

  if (filter_tf_) {
    // We create a Stamped transform from the filtered transform
    m_transform = tf_interpolation(m_transform, stampedTransform_in, 0.1);
    tf::StampedTransform stampedTransform_out(
        m_transform, transformMsg.header.stamp, transformMsg.header.frame_id,
        transformMsg.child_frame_id);
    // Filtered transformation
    m_tf_broadcaster.sendTransform(stampedTransform_out);
  } else {
    // We broadcast the transformations to the TF Server
    // Original transformation without filter
    m_tf_broadcaster.sendTransform(stampedTransform_in);
  }

  // test if we have both marker transformations ready
  //ros::spinOnce();
  bool tf_marker1_available, tf_marker2_available;
  uav_base_link_ = "cam";
  tf_marker1_available = m_tf_listener.canTransform(uav_base_link_, ugv_marker_frames_[0], transformMsg.header.stamp, NULL);
  tf_marker2_available = m_tf_listener.canTransform(uav_base_link_, ugv_marker_frames_[1], transformMsg.header.stamp, NULL);

  if (tf_marker1_available && tf_marker2_available) {
    calculate_target_pose();
  }

  // If we have 4 markers we are expecting four messages like this
  //is this the first one?
  if(number_of_received_markers_ == 0){
    //ok then check the name in the map
    check_received_markers_[transformMsg.child_frame_id] = true;
    number_of_received_markers_ = 1;
    time_last_tf_message_ = transformMsg.header.stamp;
    //check the amount of expected markers, if it is the last one, breaks)
    if(number_of_received_markers_ == n_ugv_){
      clean_marker_vars();
      //ros::spinOnce();
      calculate_target_pose();
      return;
    }
  }
  else{
    // It is not the first marker
    // Check if the time stamps are equal (same group of detection has same time stamps)
    if (time_last_tf_message_ == transformMsg.header.stamp) {
      // A new marker from the same image has arrived
      //Check that it is not repeated in the map
      if (check_received_markers_[transformMsg.child_frame_id] == true){
        ROS_ERROR("Received a repeated frame!!!");
      } else {
        check_received_markers_[transformMsg.child_frame_id] = true;
      }
      number_of_received_markers_++;
      if(number_of_received_markers_ == n_ugv_){
        clean_marker_vars();
        //ros::spinOnce();
        calculate_target_pose();
        return;
      }
    }
    else{
      //That means only one was received
      // say error! only one received
      clean_marker_vars();
      //check the new marker in the map
      check_received_markers_[transformMsg.child_frame_id] = true;
      number_of_received_markers_ = 1;
      time_last_tf_message_ = transformMsg.header.stamp;
      if(number_of_received_markers_ == n_ugv_){
        clean_marker_vars();
        //ros::spinOnce();
        calculate_target_pose();
        return;
      }
    }
  }
}

void DefineTargetNode::clean_marker_vars(void){
  //clear variables
  number_of_received_markers_ = 0;
  for (int i=0; i<n_ugv_; i++){
    check_received_markers_[ugv_marker_frames_[i]] = false;
  }
  time_last_tf_message_ = ros::Time(0);
}

void DefineTargetNode::calculate_target_pose(void) {
  // Now we want to obtain a transformation from the Quadcopter frame to the
  // Marker Frame
  geometry_msgs::PoseStamped pose_marker_in_marker_frame, pose_marker_in_quad_frame[n_ugv_];
  geometry_msgs::PointStamped center_marker;
  tf::Vector3 marker_position_vector[n_ugv_];
  tf::Transform marker_tf[n_ugv_];
  // Messages are always initialized with zero/false values
  // A properly formed empty quaternion needs w=1.0
  pose_marker_in_marker_frame.pose.orientation.w = 1.0;

  for (int i; i< n_ugv_; i++){
    pose_marker_in_marker_frame.header.frame_id = ugv_marker_frames_[i];
    try {
      m_tf_listener.transformPose(uav_base_link_, pose_marker_in_marker_frame,
                                   pose_marker_in_quad_frame[i]);
      // m_tf_listener.lookupTransform(ugv_marker_frame_[n_ugv_], uav_base_link_,
      //                              ros::Time(0), marker_to_base_tf[n_ugv_]);
      tf::poseMsgToTF(pose_marker_in_quad_frame[i].pose, marker_tf[i]);
      marker_tf[i].getOrigin();
      ROS_INFO("%s position in quadcopter frame: (%f, %f, %f)",
               ugv_marker_frames_[i].c_str(), marker_tf[i].getOrigin().x(),
               marker_tf[i].getOrigin().y(), marker_tf[i].getOrigin().z());
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
    }
  }

  // Target is the middle point of all the markers (only two for now)
//  double x, y, z;
//  x = (marker_position_vector[0].point.x + marker_position_vector[1].point.x) / 2.0;
//  y = (marker_position_vector[0].point.y + marker_position_vector[1].point.y) / 2.0;
//  z = (marker_position_vector[0].point.z + marker_position_vector[1].point.z) / 2.0;




  tf::Vector3 center_between_markers = vector_interpolation(
      marker_tf[0].getOrigin(), marker_tf[1].getOrigin(), 0.5);
  tf::Quaternion interpolated_quaternion = quaternion_interpolation(
      marker_tf[0].getRotation(), marker_tf[1].getRotation(), 0.5);
  //tf::Vector3 target_position(x, y, z);
  ROS_INFO("Position in quadcopter frame: (%f, %f, %f)", center_between_markers.x(),center_between_markers.y(), center_between_markers.z());

  // Interpolate transforms


  //!The output of this node should be a new pose in Quadcopter base_link frame.
}

tf::Transform& DefineTargetNode::tf_interpolation(tf::Transform& new_tf,
                                           const tf::Transform& old_tf, double interpolation_weight) {
  //! Taken from ar_sys system_viewer
  //! A digital filter for the detection.
  // It works but we found some singularities in the borders with some
  // inclinations.
  // We changed the digital_filter_change_rate variable from 0.5 to 0.1 and it
  // works better
  tf::Vector3 pos = vector_interpolation(new_tf.getOrigin(), old_tf.getOrigin(), interpolation_weight);
  new_tf.setOrigin(pos);

  // I modified this in order to avoid singularities in angles
  tf::Quaternion orn = quaternion_interpolation(new_tf.getRotation(), old_tf.getRotation(), interpolation_weight);
  new_tf.setRotation(orn);
  return new_tf;
}

tf::Vector3 DefineTargetNode::vector_interpolation(
    tf::Vector3 new_vector, tf::Vector3 old_vector,
    double interpolation_weight) {
  tf::Vector3 interpolated_vector((1 - interpolation_weight) * old_vector.x() +
                                       interpolation_weight * new_vector.x(),
                                  (1 - interpolation_weight) * old_vector.y() +
                                       interpolation_weight * new_vector.y(),
                                  (1 - interpolation_weight) * old_vector.z() +
                                       interpolation_weight * new_vector.z());
  return interpolated_vector;
}

tf::Quaternion DefineTargetNode::quaternion_interpolation(
    tf::Quaternion new_q, tf::Quaternion old_q,
    double interpolation_weight){
  tf::Quaternion interpolated_q;
  interpolated_q = old_q.slerp(new_q, interpolation_weight);
  return interpolated_q;
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
