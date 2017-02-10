#include "tud_coop_uv/definetargetnode.hpp"

DefineTargetNode::DefineTargetNode() : number_of_received_markers_(0) {
  // if only one ARSYS marker
  time_last_tf_message_ = ros::Time::now();
  //default names initialization
  // This should be the names of the markerboards in arsys
  // It is also the name of the coordinate frames of each marker

  default_ugv_robot_names_.push_back("c3po");
  default_ugv_robot_names_.push_back("r2d2");
  default_ugv_robot_names_.push_back("undefined_robot");
  default_ugv_robot_names_.push_back("undefined_robot");
  default_ugv_robot_names_.push_back("undefined_robot");
  default_ugv_robot_names_.push_back("undefined_robot");

  //! UGV stands for unmanned ground vehicle
  //! UAV stands for unmanned aerial vehicle

  // Parameters definition
  std::string marker_tf_topic, marker_pose_topic;

  nh_.param<std::string>("marker_tf_topic", marker_tf_topic,
                         "/ar_multi_boards/transform");

  nh_.param<std::string>("marker_pose_topic", marker_pose_topic,
                         "/ar_multi_boards/pose");

  nh_.param<std::string>("uav_base_link_", uav_base_link_,
                         "/ardrone_base_link");


  //! TODO(racuna) fix filtering
  nh_.param<bool>("filter_tf", filter_tf_, false);

  // Number of tracked objects to be expected (one marker or makerboard for each one)
  nh_.param<int>("n_ugv", n_ugv_, 1);
  if(n_ugv_ < 1 || n_ugv_ > 6){
    ROS_ERROR("This node works with a minimum of 1 and a maximun of 6 UGV (Unmanned Ground Vehicles)");
  }

  // Definition of UGV frames and their top_marker frames
  std::string ugv_frame;
  for (int i=0; i<n_ugv_; i++){
    nh_.param<std::string>("ugv_frame_ugv_"+i, ugv_frame, default_ugv_robot_names_[i]);
    ugv_frames_.push_back(ugv_frame);
    ugv_marker_frames_.push_back(ugv_frame + "_marker_top");
    check_received_markers_[ugv_marker_frames_[i]] = false;
  }

  arsys_transform_sub_ = nh_.subscribe(
      marker_tf_topic, 1, &DefineTargetNode::arsys_transform_callback, this);

  target_updated_pub_ = nh_.advertise<std_msgs::Empty>("/define_target/target_updated", 1);
}

void DefineTargetNode::arsys_transform_callback(
    const geometry_msgs::TransformStamped& transformMsg) {
  //! This message comes from the ar_sys package.
  //! It has the transform of a board_frame to the camera_frame
  //! ar_sys will send one message for each board detected in an image
  //! THe chidl_frame_id is the frame of the detected marker or marker board
  //! this name is configured in ar_sys using the parameters in the launch file
  //! The parent frame is the camera frame
//  tf::StampedTransform tf_base_cam = tf::StampedTransform(
//                     tf::Transform(
//                       tf::createQuaternionFromRPY(0.0, 180.0 * _DEG2RAD, 90.0 * _DEG2RAD),
//                       tf::Vector3(0.0, 0.00, -0.068)),
//                     ros::Time::now(), "ardrone_base_link", "cam");
//  m_tf_broadcaster.sendTransform(tf_base_cam);

//  ROS_INFO("Quaternion %f, %f, %f. %f", tf_base_cam.getRotation().getW(),
//           tf_base_cam.getRotation().getX(),
//           tf_base_cam.getRotation().getY(),
//           tf_base_cam.getRotation().getZ());

  // First we convert the message to a StampedTransform object
  tf::StampedTransform stampedTransform_in;  
  tf::transformStampedMsgToTF(transformMsg, stampedTransform_in);

  //ROS_INFO("Received Board %s: sec: %i, nsec: %i", transformMsg.child_frame_id.c_str(),
  //         transformMsg.header.stamp.sec, transformMsg.header.stamp.nsec);

  // Check if Id is valid
  std::string frame_id = transformMsg.child_frame_id;
  bool frame_is_wanted = false;
  for (int i=0; i<n_ugv_; i++){
    if(ugv_marker_frames_[i] == frame_id){
      frame_is_wanted = true;
    }
  }
  if(!frame_is_wanted){
    // If the frame is not in the list of wanted frames then we dont process this transform.
    return;
  }

  if (filter_tf_) {
    //! TODO(racuna) fix this part so it is possible to use several markers
    //! We will need a past value for each configured marker
    // We filter using past value and new value
    m_transform = tf_interpolation(m_transform, stampedTransform_in, 0.1);
    tf::StampedTransform stampedTransform_out(
        m_transform, transformMsg.header.stamp, transformMsg.header.frame_id,
        transformMsg.child_frame_id);
    // We broadcast the transformations to the TF Server
    // Filtered transformation
    m_tf_broadcaster.sendTransform(stampedTransform_out);
  } else {
    // We broadcast the transformations to the TF Server
    // Original transformation without filter
    //m_tf_broadcaster.sendTransform(stampedTransform_in);
  }

  //! When we are checking for more than one marker (to do joint tracking) then
  //! we need to check if all the markers are available in a particular image
  //! we compare all the frames with the required ones
  // is this the first marker?
  if(number_of_received_markers_ == 0){
    // then check the name in the map
    check_received_markers_[transformMsg.child_frame_id] = true;
    number_of_received_markers_ = 1;
    time_last_tf_message_ = transformMsg.header.stamp;
    // check the amount of expected markers, if it is the last one, breaks)
    if(number_of_received_markers_ == n_ugv_){
      clean_marker_vars();
      calculate_target_pose();
      return;
    }
  }
  else{
    // It is not the first marker
    // Check if the time stamps is equal to the las received marker
    // (markers detected in same image have same time stamps)
    if (time_last_tf_message_ == transformMsg.header.stamp) {
      // A new marker from the same image has arrived
      // Check that it is not repeated in the map (foolproof)
      if (check_received_markers_[transformMsg.child_frame_id] == true){
        ROS_ERROR("Received a repeated frame!!!");
      } else {
        check_received_markers_[transformMsg.child_frame_id] = true;
      }
      number_of_received_markers_++;
      if(number_of_received_markers_ == n_ugv_){
        clean_marker_vars();
        calculate_target_pose();
        return;
      }
    }
    else{
      ROS_ERROR_ONCE("The system is defined for %i UGVs but only %i markers were detected in the image", n_ugv_, number_of_received_markers_);
      ROS_DEBUG("The system is defined for %i UGVs but only %i markers were detected in the image", n_ugv_, number_of_received_markers_);
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
  tf::StampedTransform target_stamped_transform;
  geometry_msgs::PoseStamped pose_marker_in_marker_frame, pose_marker_in_quad_frame[n_ugv_];
  tf::Transform marker_tf[n_ugv_];
  // Messages are always initialized with zero/false values
  // A properly formed empty quaternion needs w=1.0
  pose_marker_in_marker_frame.pose.orientation.w = 1.0;

  for (int i=0; i< n_ugv_; i++){
    pose_marker_in_marker_frame.header.frame_id = ugv_marker_frames_[i];
    try {
      m_tf_listener.transformPose(uav_base_link_, pose_marker_in_marker_frame,
                                   pose_marker_in_quad_frame[i]);
      tf::poseMsgToTF(pose_marker_in_quad_frame[i].pose, marker_tf[i]);
      marker_tf[i].getOrigin();
      //ROS_INFO("%s position in quadcopter frame: (%f, %f, %f)",
      //         ugv_marker_frames_[i].c_str(), marker_tf[i].getOrigin().x(),
      //         marker_tf[i].getOrigin().y(), marker_tf[i].getOrigin().z());
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
    }
  }

  // Target is the middle point of all the markers (only two for now)
  tf::Vector3 average_pos;
  tf::Quaternion average_q;
  for(int i=0; i<n_ugv_; i++){
    if(i==0){
      average_pos = marker_tf[i].getOrigin();
      average_q = marker_tf[i].getRotation();
    }
    else{
      average_pos = vector_interpolation(average_pos, marker_tf[i].getOrigin(), 0.5);
      average_q = quaternion_interpolation(average_q, marker_tf[i].getRotation(), 0.5);
    }
  }
//  tf::Vector3 center_between_markers = vector_interpolation(
//      marker_tf[0].getOrigin(), marker_tf[1].getOrigin(), 0.5);
//  tf::Quaternion interpolated_quaternion = quaternion_interpolation(
//      marker_tf[0].getRotation(), marker_tf[1].getRotation(), 0.5);
  //tf::Vector3 target_position(x, y, z);
  //ROS_INFO("Position in quadcopter frame: (%f, %f, %f)", average_pos.x(),average_pos.y(), average_pos.z());

  // We publish the transformation to tf tree
  target_stamped_transform.child_frame_id_ = "tracking_target";
  target_stamped_transform.frame_id_ = uav_base_link_;
  target_stamped_transform.stamp_ = ros::Time::now();
  target_stamped_transform.setOrigin(average_pos);
  target_stamped_transform.setRotation(average_q);

  m_tf_broadcaster.sendTransform(target_stamped_transform);


  //!The output of this is a new target transformation from Quadcopter base_link frame.
  //!
  //!

  //We now broadcast a signal that we succesfully received a processed a target marker
  std_msgs::Empty empty;
  target_updated_pub_.publish(empty);
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
