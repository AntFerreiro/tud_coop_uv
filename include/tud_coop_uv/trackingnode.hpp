#ifndef TRACKINGNODE_HPP
#define TRACKINGNODE_HPP

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_listener.h"
#include "tud_coop_uv/filter.hpp"

#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"

#include <dynamic_reconfigure/server.h>
#include <tud_coop_uv/tracking_dynamic_param_configConfig.h>



class TrackingNode
{
public:
    TrackingNode();
    ros::NodeHandle nh_;

    // ROS message callbacks

    void arsys_transform_callback(const geometry_msgs::TransformStamped& transformMsg);
    void update_control(const std_msgs::Empty empty_msg);


    // ROS dynamic reconfigure callback
    void dynamicReconfigureCb(
        tud_coop_uv::tracking_dynamic_param_configConfig& config, uint32_t level);

    // tracking_node functions
    bool get_target_pose(tf::Pose& target_pose);
    void tracking_control(tf::Pose& target_pose);

    //Velocity based tracking control (to be used with ardrone_velocity)
    void velxy_control(tf::Pose& target_pose, double& velx, double& vely);

    //Direct tilt control (without ardrone_velocity, commands direct to ardrone autonomy)
    void tiltxy_control(tf::Pose& target_pose, double& tiltx, double& tilty, ros::Duration dt);

    //Height control is always a velocity control
    void height_control(tf::Pose& target_pose, double& velz, ros::Duration dt);

    void i_term_increase(double& i_term, double new_err, double cap);

    void set_hover(void);
    void draw_arrow_rviz(tf::Vector3& endpoint);

private:
    ros::Subscriber arsys_pose_sub_;
    ros::Subscriber marker_transform_sub_;
    ros::Subscriber target_updated_sub_;

    ros::Publisher cmd_vel_pub_;
    ros::Publisher cmd_vel_marker_pub_; //! For debugging cmd_vel in RVIZ
    ros::Publisher ardrone_land_pub_;
    ros::Publisher debug_pub_; //! For debugging variables in rqt_plot

    nav_msgs::Odometry odo_msg_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;

    Filter derivative_filter_x;
    Filter derivative_filter_y;

    // dynamic reconfigure server
    dynamic_reconfigure::Server<tud_coop_uv::tracking_dynamic_param_configConfig>
        server_;

    double ref_quadcopter_height_;

    double max_linear_velocity_, max_yaw_velocity_, max_z_velocity_;
    double kp_velxy_, ki_velxy_, kd_velxy_;

    double error_x_filtered_;
    double error_y_filtered_;
    double last_error_x_;
    double last_error_y_;
    double last_error_z_;
    double i_term_x_;
    double i_term_y_;
    double i_term_z_;

    double kp_tilt_, ki_tilt_, kd_tilt_;
    double kp_z_, ki_z_, kd_z_;
    double kp_yaw_, ki_yaw_, kd_yaw_;

    ros::Time t;
    ros::Time old_t;

    enum height_control_type_enum_ {marker_height_control, ultrasound_height_control, kf_height_control};
    height_control_type_enum_  height_control_type_ = marker_height_control;
};

#endif // TRACKINGNODE_HPP
