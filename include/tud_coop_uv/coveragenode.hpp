#ifndef COVERAGENODE_HPP
#define COVERAGENODE_HPP

#include "ros/ros.h"
#include "tf2/buffer_core.h"
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include "angles/angles.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/intersections.h>


typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Segment_3 Segment_3;
typedef Kernel::Plane_3 Plane_3;
typedef Kernel::Direction_3 Direction_3;
typedef Kernel::Ray_3 Ray_3;
typedef Kernel::Vector_3 Vector_3;



class CoverageNode
{
public:
    CoverageNode();
    ros::NodeHandle nh;

    // ROS message callbacks
//    void quad_OdomCallback(const nav_msgs::Odometry& odo_msg);
//    void arsys_marker_pose_callback(const geometry_msgs::PoseStamped& marker_pose_msg);
//    void arsys_transform_callback(const geometry_msgs::TransformStamped& transformMsg);

//    tf::TransformListener m_tf_listener;
//    tf::Transform& m_tf_digital_filter(tf::Transform &dst, const tf::Transform &src);

//    void tracking_control(tf::Vector3& tracking_point);
//    void set_hover(void);
//    void draw_arrow_rviz(tf::Vector3& endpoint);
//    bool m_cmd_valid = false;
    void draw_lines(void);
    geometry_msgs::Point calc_cam_ray(Plane_3 plane, Point_3 cam_origin, double angle_x, double angle_y);

private:
//    ros::Subscriber m_quad_vel_sub;
//    ros::Subscriber m_arsys_pose_sub;
//    ros::Subscriber m_arsys_transform_sub;
//    ros::Subscriber m_transform_sub;
//    ros::Publisher m_cmd_vel_pub;
    ros::Publisher rviz_marker_pub; //! For debugging cmd_vel in RVIZ
//    ros::Publisher m_debug_pub; //! For debugging variables in rqt_plot
//    geometry_msgs::Twist m_current_command;
//    nav_msgs::Odometry m_odo_msg;
//    tf::TransformBroadcaster m_tf_broadcaster;
//    //Initial transform for filter
//    tf::Transform m_transform = tf::Transform::getIdentity(); //Initial position
    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener* m_tf_listener;

};

#endif // COVERAGENODE_HPP
