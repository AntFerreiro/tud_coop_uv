#ifndef DEFINETARGETNODE_HPP
#define DEFINETARGETNODE_HPP

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_listener.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"

class DefineTargetNode {
 public:
  DefineTargetNode();
  ros::NodeHandle nh_;

  // ROS message callbacks
  void quad_OdomCallback(const nav_msgs::Odometry& odo_msg);

  void arsys_transform_callback(
      const geometry_msgs::TransformStamped& transformMsg);

  tf::Transform& tf_interpolation(tf::Transform& dst, const tf::Transform& src,
                                  double interpolation_weight);
  tf::Vector3 vector_interpolation(tf::Vector3 new_vector,
                                   tf::Vector3 old_vector,
                                   double interpolation_weight);
  tf::Quaternion quaternion_interpolation(tf::Quaternion new_q,
                                          tf::Quaternion old_q,
                                          double interpolation_weight);

  void calculate_target_pose(void);
  void tracking_control(tf::Vector3& tracking_point);
  void set_hover(void);
  void draw_arrow_rviz(tf::Vector3& endpoint);

  void clean_marker_vars(void);

 private:
  ros::Subscriber arsys_transform_sub_;
  ros::Publisher cmd_vel_marker_pub_;  //! For debugging cmd_vel in RVIZ
  tf::TransformBroadcaster m_tf_broadcaster;
  tf::TransformListener m_tf_listener;

  // Initial transform for filter
  tf::Transform m_transform = tf::Transform::getIdentity();  // Initial position

  // ROS Node parameters
  int n_ugv_;
  bool filter_tf_;
  // UGV default transformation frames
  std::vector<std::string> default_ugv_robot_names_;
  // UGV
  std::vector<std::string> ugv_frames_;
  std::vector<std::string> ugv_marker_frames_;
  std::string uav_base_link_;
  // UAV default transformation frame
  std::string const default_uav_frame_;
  // UAV camera frame
  std::string uav_camera_frame_;

  // helper variables
  ros::Time time_last_tf_message_;
  int number_of_received_markers_;
  std::map<std::string, bool> check_received_markers_;

  /// Maybe remove CHECK!
  bool m_cmd_valid_ = false;
  ros::Subscriber m_quad_vel_sub;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher m_debug_pub;  //! For debugging variables in rqt_plot
  geometry_msgs::Twist m_current_command;
  nav_msgs::Odometry m_odo_msg;
};

#endif  // DEFINETARGETNODE_HPP
