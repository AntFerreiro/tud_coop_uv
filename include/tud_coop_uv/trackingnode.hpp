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
    ros::NodeHandle nh_;

    // ROS message callbacks
    void quad_OdomCallback(const nav_msgs::Odometry& odo_msg);
    void arsys_marker_pose_callback(const geometry_msgs::PoseStamped& marker_pose_msg);
    void arsys_transform_callback(const geometry_msgs::TransformStamped& transformMsg);

    // tracking_node functions
    bool get_target_pose(tf::Pose& target_pose);
    void tracking_control(tf::Pose target_pose);
    void set_hover(void);
    void draw_arrow_rviz(tf::Vector3& endpoint);

private:
    ros::Subscriber quad_odom_sub_;
    ros::Subscriber arsys_pose_sub_;
    ros::Subscriber marker_transform_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher cmd_vel_marker_pub_; //! For debugging cmd_vel in RVIZ
    ros::Publisher debug_pub_; //! For debugging variables in rqt_plot
    nav_msgs::Odometry odo_msg_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;

    double ref_quadcopter_height_;
};

#endif // TRACKINGNODE_HPP
