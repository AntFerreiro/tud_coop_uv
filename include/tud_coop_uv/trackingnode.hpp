#ifndef TRACKINGNODE_HPP
#define TRACKINGNODE_HPP

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"



class TrackingNode
{
public:
    TrackingNode();
    ros::NodeHandle nh;

    // ROS message callbacks
    void quad_OdomCallback(const nav_msgs::Odometry& odo_msg);
    void arsys_marker_pose_callback(const geometry_msgs::PoseStamped& marker_pose_msg);
    void arsys_transform_callback(const geometry_msgs::TransformStamped& transformMsg);

    tf::TransformListener m_tf_listener;
    tf::Transform& m_tf_digital_filter(tf::Transform &dst, const tf::Transform &src);

    void tracking_control(tf::Vector3& tracking_point);
    void set_hover(void);
    void draw_arrow_rviz(tf::Vector3& endpoint);
    bool m_cmd_valid = false;

private:
    ros::Subscriber m_quad_vel_sub;
    ros::Subscriber m_arsys_pose_sub;
    ros::Subscriber m_arsys_transform_sub;
    ros::Subscriber m_transform_sub;
    ros::Publisher m_cmd_vel_pub;
    ros::Publisher m_cmd_vel_marker_pub; //! For debugging cmd_vel in RVIZ
    ros::Publisher m_debug_pub; //! For debugging variables in rqt_plot
    geometry_msgs::Twist m_current_command;
    nav_msgs::Odometry m_odo_msg;
    tf::TransformBroadcaster m_tf_broadcaster;
    //Initial transform for filter
    tf::Transform m_transform = tf::Transform::getIdentity(); //Initial position
};

#endif // TRACKINGNODE_HPP
