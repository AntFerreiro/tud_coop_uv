#include "tud_coop_uv/coveragenode.hpp"


CoverageNode::CoverageNode()
{

//    m_transform_sub = nh.subscribe ("/arsys_single_board/transform", 1, &TrackingNode::arsys_transform_callback, this);
//    m_quad_vel_sub = nh.subscribe("/ardrone/odometry",1,&TrackingNode::quad_OdomCallback,this,ros::TransportHints().tcpNoDelay());
//    m_arsys_pose_sub = nh.subscribe("/ar_single_board/pose", 1, &TrackingNode::arsys_marker_pose_callback, this);

//    m_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/tracking/cmd_vel", 1);
//    m_debug_pub = nh.advertise<std_msgs::Float64>("/pid/debug",1);

    m_tf_listener = new tf2_ros::TransformListener(m_tfBuffer);
    rviz_marker_pub = nh.advertise<visualization_msgs::Marker>("/cmd_vel_marker", 1);
}


geometry_msgs::Point CoverageNode::calc_cam_ray(Plane_3 plane, Point_3 cam_origin, double angle_x, double angle_y){
    geometry_msgs::TransformStamped transform_msg_ardrone_board, in; // boardfiltered -> ardrone_base_bottomcam
    try{
      transform_msg_ardrone_board = m_tfBuffer.lookupTransform("boardFiltered", "ardrone_base_bottomcam", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
    }

    //We convert the transform message into tf2::transform
    tf2::Transform transform;
    tf2::fromMsg(transform_msg_ardrone_board.transform, transform);


    //Ground plane in world coordinates
    //Plane is defined by a point and normal
    tf2::Vector3 normal(0,0,1);

    tf2::Vector3 p_in_plane1(0,0,0);
    tf2::Vector3 p_in_plane2(0,1,0);
    tf2::Vector3 p_in_plane3(1,0,0);

    normal = transform.inverse()*normal;

    p_in_plane1 = transform.inverse()*p_in_plane1;
    p_in_plane2 = transform.inverse()*p_in_plane2;
    p_in_plane3 = transform.inverse()*p_in_plane3;
    //int i;
    //for(i=0;i<25344;i++){
    //    p_in_plane3 = transform.inverse()*p_in_plane3;
    //}

    //ROS_INFO("Normal: (%f, %f, %f)",p_in_plane.x(),p_in_plane.y(),p_in_plane.z());

    Direction_3 normal_cgal(normal.x(),normal.y(),normal.z());
    Point_3 point_plane_cgal1(p_in_plane1.x(),p_in_plane1.y(),p_in_plane1.z());
    Point_3 point_plane_cgal2(p_in_plane2.x(),p_in_plane2.y(),p_in_plane2.z());
    Point_3 point_plane_cgal3(p_in_plane3.x(),p_in_plane3.y(),p_in_plane3.z());
    //Plane_3 plane2(point_plane_cgal,normal_cgal);
    Plane_3 plane2(point_plane_cgal1,point_plane_cgal2,point_plane_cgal3);


    //normal = transform_ardrone_board.transform()*normal;

    tf2::Vector3 x_axis(1, 0, 0);
    tf2::Vector3 y_axis(0, 1, 0);
    tf2::Vector3 z_axis(0, 0, 1);

    tf2::Vector3 fov_vector_x = z_axis.rotate(y_axis,angles::from_degrees(-angle_x));
    tf2::Vector3 fov_vector_y = z_axis.rotate(x_axis,angles::from_degrees(+angle_y));
    tf2::Vector3 fov = fov_vector_x + fov_vector_y;

    //Now we use CGAL library to calculate intersections
    Vector_3 fov_result_cgal(fov.x(),fov.y(),fov.z());
    Ray_3 cam_ray(cam_origin,fov_result_cgal);

    // This result will be an CGAL::Object
    auto inter_result = CGAL::intersection(plane2, cam_ray);

    Point_3 point_result;
    Segment_3 segment_result;

    //We switch back to ROS point format
    geometry_msgs::Point end_point_ray;

    // Now we have to convert the Object to a point or segment
    if (assign(point_result, inter_result)) {
      //Use the point from CGAL to fill the point in ROS
        end_point_ray.x = point_result.x();
        end_point_ray.y = point_result.y();
        end_point_ray.z = point_result.z();

        return end_point_ray;
      std::cout << "Interseccion linea plano: " << point_result << std::endl;
    } else if (assign(segment_result, inter_result)) {
      // do something with segment
        return end_point_ray;
    }
}


void CoverageNode::draw_lines(void){
    //We define the world plane
    Point_3 p(0,0,1);
    Direction_3 normal(0,0,1);
    Plane_3 plane(p,normal);

    //Camera coordinates (later this will (0,0,0) in the bottom camera frame
    Point_3 cam_origin(0,0,0);

    // Field of view of the camera
    double fov_camera_x = 47.5;
    double fov_camera_y = 36.5;

    //double fov_camera_x = 60;
    //double fov_camera_y = 10;

    //We create two vectors
    // each one with the field of view angle in that axis
    // then we add each vector to obtain the vector for that camera corner
    //Corner 1: First quadrant
    //Negative x rotation | Positive y rotation
    geometry_msgs::Point end_point_ray1 = calc_cam_ray(plane, cam_origin, -fov_camera_x, +fov_camera_y);

    //Corner 2: Second quadrant
    //Negative x rotation | Negative y rotation
    geometry_msgs::Point end_point_ray2 = calc_cam_ray(plane, cam_origin, -fov_camera_x, -fov_camera_y);

    //Corner 3: Third quadrant
    //Positive x rotation | Negative y rotation
    geometry_msgs::Point end_point_ray3 = calc_cam_ray(plane, cam_origin, +fov_camera_x, -fov_camera_y);

    //Corner 4: Fourth quadrant
    //Positive x rotation | Positive y rotation
    geometry_msgs::Point end_point_ray4 = calc_cam_ray(plane, cam_origin, +fov_camera_x, +fov_camera_y);

    // Definition of the line list marker for RVIZ
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/ardrone_base_bottomcam";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "tud_coop_uv_coverage";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 0;
    line_list.scale.x = 0.01;
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;

    //Origin of all the camera Rays
    geometry_msgs::Point start;
    start.x = cam_origin.x();
    start.y = cam_origin.y();
    start.z = cam_origin.z();

    //We add the points of each ray
    line_list.points.push_back(start);
    line_list.points.push_back(end_point_ray1);

    line_list.points.push_back(start);
    line_list.points.push_back(end_point_ray2);

    line_list.points.push_back(start);
    line_list.points.push_back(end_point_ray3);

    line_list.points.push_back(start);
    line_list.points.push_back(end_point_ray4);

    rviz_marker_pub.publish(line_list);
}





