#include <iostream>
#include "tud_coop_uv/coveragenode.hpp"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/intersections.h>
#include "angles/angles.h"
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Segment_3 Segment_3;
typedef Kernel::Plane_3 Plane_3;
typedef Kernel::Direction_3 Direction_3;
typedef Kernel::Ray_3 Ray_3;
typedef Kernel::Vector_3 Vector_3;


CoverageNode::CoverageNode()
{

//    m_transform_sub = nh.subscribe ("/arsys_single_board/transform", 1, &TrackingNode::arsys_transform_callback, this);
//    m_quad_vel_sub = nh.subscribe("/ardrone/odometry",1,&TrackingNode::quad_OdomCallback,this,ros::TransportHints().tcpNoDelay());
//    m_arsys_pose_sub = nh.subscribe("/ar_single_board/pose", 1, &TrackingNode::arsys_marker_pose_callback, this);

//    m_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/tracking/cmd_vel", 1);
//    m_debug_pub = nh.advertise<std_msgs::Float64>("/pid/debug",1);
    rviz_marker_pub = nh.advertise<visualization_msgs::Marker>("/cmd_vel_marker", 1);




}

void CoverageNode::draw_lines(void){
    tf2::Quaternion q1,q2;
    q1.setRPY(angles::from_degrees(45),0,0);
    q2.setRPY(0,0,angles::from_degrees(45));
//    q1.setEuler(0,angles::from_degrees(45),angles::from_degrees(45));
    tf2::Vector3 vector(0, 0, 1);
    tf2::Vector3 rotated_vector = tf2::quatRotate(q1,vector);
    rotated_vector = tf2::quatRotate(q2,rotated_vector);

    rotated_vector = vector;

    tf2::Vector3 x_axis(1, 0, 0);
    tf2::Vector3 y_axis(0, 1, 0);
    tf2::Vector3 z_axis(0, 0, 1);
    rotated_vector = rotated_vector.rotate(x_axis,angles::from_degrees(-45));
    rotated_vector = rotated_vector.rotate(z_axis,angles::from_degrees(-45));


    Point_3 p(0,0,0);
    Direction_3 normal(0,0,1);
    Plane_3 plane(p,normal);




    Point_3 cam_origin(0,0,-1);
    Direction_3 cam_ray_dir(rotated_vector.x(),rotated_vector.y(),rotated_vector.z());
    Ray_3 cam_ray(cam_origin,cam_ray_dir);

    Direction_3 test_dir(1,1,1);
    Ray_3 cam_ray2(cam_origin,test_dir);

    //Otra ṕrueba mas
    rotated_vector = vector.rotate(x_axis,angles::from_degrees(60));
    Vector_3 test_dir1(rotated_vector.x(),rotated_vector.y(),rotated_vector.z());
    rotated_vector = vector.rotate(y_axis,angles::from_degrees(45));
    Vector_3 test_dir2(rotated_vector.x(),rotated_vector.y(),rotated_vector.z());
    Vector_3 test_dir3 = test_dir1 +test_dir2;
    Ray_3 cam_ray3(cam_origin,test_dir3);


    // This result will be an CGAL::Object
    auto result2 = CGAL::intersection(plane, cam_ray3);

    Point_3 point_result;
    Segment_3 segment_result;
    // Now we have to convert the Object to a point or segment
    if (assign(point_result, result2)) {
      //do something with point
      std::cout << "Interseccion linea plano: " << point_result << std::endl;
    } else if (assign(segment_result, result2)) {
      // do something with segment
    }


    geometry_msgs::Point start_p,end_p;
    start_p.x = cam_origin.x();
    start_p.y = cam_origin.y();
    start_p.z = cam_origin.z();

    end_p.x = point_result.x();
    end_p.y = point_result.y();
    end_p.z = point_result.z();

    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/my_frame";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "tud_coop_uv";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 0;
    line_list.scale.x = 0.01;
    // Line strip is blue
    line_list.color.r = 1.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;
    line_list.color.a = 1.0;

    line_list.points.push_back(start_p);
    line_list.points.push_back(end_p);



    visualization_msgs::Marker cmd_vel_marker;
    cmd_vel_marker.header.frame_id = "/my_frame";
    cmd_vel_marker.header.stamp = ros::Time::now();
    cmd_vel_marker.ns = "tud_coop_uv";
    cmd_vel_marker.id = 0;
    cmd_vel_marker.type = visualization_msgs::Marker::ARROW;
    cmd_vel_marker.action = visualization_msgs::Marker::ADD;
    cmd_vel_marker.scale.x = 0.1;
    cmd_vel_marker.scale.y = 0.1;
    cmd_vel_marker.scale.z = 0.1;
    cmd_vel_marker.color.a = 1.0; // Don't forget to set the alpha!
    cmd_vel_marker.color.r = 0.0;
    cmd_vel_marker.color.g = 1.0;
    cmd_vel_marker.color.b = 0.0;

    //We define the start and end points of the ARROW
    geometry_msgs::Point p1;
    p1.x = 0.0;
    p1.y = 0.0;
    p1.z = 0.0;
    cmd_vel_marker.points.push_back(p1);
    line_list.points.push_back(p1);
    p1.x = 2;
    p1.y = 2;
    p1.z = 0.0;
    cmd_vel_marker.points.push_back(p1);
    line_list.points.push_back(p1);
    //rviz_marker_pub.publish(cmd_vel_marker);

    rviz_marker_pub.publish(line_list);
}





