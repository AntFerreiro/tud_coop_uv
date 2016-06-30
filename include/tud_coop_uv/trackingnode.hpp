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


    tf::Transform& m_tf_digital_filter(tf::Transform &dst, const tf::Transform &src);

    tf::Point get_target_point(void);
    void tracking_control(tf::Vector3& tracking_point);
    void set_hover(void);
    void draw_arrow_rviz(tf::Vector3& endpoint);
    bool m_cmd_valid = false;

private:
    ros::Subscriber m_quad_vel_sub;
    ros::Subscriber m_arsys_pose_sub;
    ros::Subscriber arsys_transform_sub_;
    ros::Subscriber m_transform_sub;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher cmd_vel_marker_pub_; //! For debugging cmd_vel in RVIZ
    ros::Publisher debug_pub_; //! For debugging variables in rqt_plot
    geometry_msgs::Twist m_current_command;
    nav_msgs::Odometry odo_msg_;
    tf::TransformBroadcaster m_tf_broadcaster;
    tf::TransformListener tf_listener_;
    //Initial transform for filter
    tf::Transform m_transform = tf::Transform::getIdentity(); //Initial position
};

#endif // TRACKINGNODE_HPP
