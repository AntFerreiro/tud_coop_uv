#include "tud_coop_uv/trackingnode.hpp"

TrackingNode::TrackingNode()
{

    m_transform_sub = nh.subscribe ("/arsys_single_board/transform", 1, &TrackingNode::arsys_transform_callback, this);
    m_quad_vel_sub = nh.subscribe("/ardrone/odometry",1,&TrackingNode::quad_OdomCallback,this,ros::TransportHints().tcpNoDelay());
    m_arsys_pose_sub = nh.subscribe("/ar_single_board/pose", 1, &TrackingNode::arsys_marker_pose_callback, this);

    m_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/tracking/cmd_vel", 1);
    m_debug_pub = nh.advertise<std_msgs::Float64>("/pid/debug",1);    
    m_cmd_vel_marker_pub = nh.advertise<visualization_msgs::Marker>("/cmd_vel_marker", 1);
}

void TrackingNode::quad_OdomCallback(const nav_msgs::Odometry& odo_msg){
    std_msgs::Float64 debug_msg;
    m_odo_msg = odo_msg;
    //debug_msg.data = m_filtered_vel_x;
    //m_debug_pub.publish(debug_msg);
}

void TrackingNode::set_hover(void){
    geometry_msgs::Twist cmd_vel_out;
    cmd_vel_out.linear.x = 0;
    cmd_vel_out.linear.y = 0;
    cmd_vel_out.linear.z = 0;
    cmd_vel_out.angular.x = 0;
    cmd_vel_out.angular.y = 0;
    cmd_vel_out.angular.z = 0;
    m_cmd_vel_pub.publish(cmd_vel_out);
}

void TrackingNode::tracking_control(tf::Vector3& tracking_point){
    // In the frame /ardrone_base_link_fixed
    double velx;
    double vely;

    velx = tracking_point.x()*0.75;
    vely = tracking_point.y()*0.75;

    double norm = sqrt(velx*velx + vely*vely);
    //!Completly arbitrary number (change with ros_param)
    if (norm > 0.4){
        velx = (velx/norm)*0.4;
        vely = (vely/norm)*0.4;
    }

    geometry_msgs::Twist cmd_vel_out;
    cmd_vel_out.linear.x = velx;
    cmd_vel_out.linear.y = vely;
    m_cmd_vel_pub.publish(cmd_vel_out);
}

void TrackingNode::draw_arrow_rviz(tf::Vector3& endpoint){
    // Publishes a Marker (ARROW) in RVIZ
    // from the center of the quadcopter frame to the center of the AruCo Board frame
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
    cmd_vel_marker.color.a = 1.0; // Don't forget to set the alpha!
    cmd_vel_marker.color.r = 0.0;
    cmd_vel_marker.color.g = 1.0;
    cmd_vel_marker.color.b = 0.0;

    //We define the start and end points of the ARROW
    geometry_msgs::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    cmd_vel_marker.points.push_back(p);
    p.x = endpoint.x();
    p.y = endpoint.y();
    p.z = 0.0;
    cmd_vel_marker.points.push_back(p);
    m_cmd_vel_marker_pub.publish(cmd_vel_marker);
}

void TrackingNode::arsys_marker_pose_callback(const geometry_msgs::PoseStamped& marker_pose_msg){
    tf::StampedTransform transform_ardrone_board; // boardFiltered -> ardrone_base_bottomcam -> ardrone_base_link
    try{
      m_tf_listener.lookupTransform("/boardFiltered", "/ardrone_base_link", ros::Time(0), transform_ardrone_board);

    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
    }

    // Now we create a new fixed frame transformation: /ardrone_base_link_fixed
    // This will give us an ardrone frame that only changes in yaw
    // in the same position of the /ardrone_base_link frame
    double roll, pitch, yaw;
    tf::StampedTransform stampedTransform_fixed;
    tf::Transform transform_fixed;    
    transform_ardrone_board.getBasis().getRPY(roll, pitch, yaw);

    transform_fixed.setOrigin(transform_ardrone_board.getOrigin());
    transform_fixed.setRotation(tf::createQuaternionFromRPY(0.0,0.0,yaw));

    // We publish the new stamped transform for RVIZ
    stampedTransform_fixed.child_frame_id_ = "/ardrone_base_link_fixed";
    stampedTransform_fixed.frame_id_ = "/boardFiltered";
    stampedTransform_fixed.setData(transform_fixed);
    stampedTransform_fixed.stamp_ = ros::Time::now();
    m_tf_broadcaster.sendTransform(stampedTransform_fixed);

    // Obtain marker position in ardrone fixed frame
    tf::Vector3 marker_position(0,0,0);
    marker_position = stampedTransform_fixed.inverse()*marker_position;

    // We draw a tracking arrow in RVIZ for visual reference
    draw_arrow_rviz(marker_position);

    // Now we do the tracking
    tracking_control(marker_position);

    //! TODO implement orientation PID controller using yaw variable

}

void TrackingNode::arsys_transform_callback (const geometry_msgs::TransformStamped& transformMsg){
    //! This message comes from the ar_sys package.
    //! It has the transform of a single board related to the AR.Drone bottom camera

    // First we convert the message to a StampedTransform object
    tf::StampedTransform stampedTransform_in;

    tf::transformStampedMsgToTF(transformMsg, stampedTransform_in);

    //! Taken from ar_sys system_viewer
    //! A digital filter for the detection.
    // It works but we found some singularities in the borders with some inclinations,
    // perhaps due to the quaternion filter...
    // We changed the digital_filter_change_rate variable from 0.5 to 0.75 and it works better
    m_transform = m_tf_digital_filter(m_transform,stampedTransform_in);

    double x = m_transform.getOrigin().x();
    double y = m_transform.getOrigin().y();

    //ROS_INFO("Posicion Patron: (%f,%f)",x,y);

    // Now we create a Stamped transform from the filtered transform
    tf::StampedTransform stampedTransform_out(m_transform,transformMsg.header.stamp,transformMsg.header.frame_id,"/boardFiltered");

    // We broadcast the transformation information
    m_tf_broadcaster.sendTransform(stampedTransform_in);

    m_tf_broadcaster.sendTransform(stampedTransform_out);
}


tf::Transform& TrackingNode::m_tf_digital_filter(tf::Transform &dst, const tf::Transform &src)
{
    double digital_filter_change_rate =0.1;

    tf::Vector3 posOld = dst.getOrigin();
    tf::Vector3 posNew = src.getOrigin();
    tf::Vector3 pos
    (
        (1 - digital_filter_change_rate) * posOld.x() + digital_filter_change_rate * posNew.x(),
        (1 - digital_filter_change_rate) * posOld.y() + digital_filter_change_rate * posNew.y(),
        (1 - digital_filter_change_rate) * posOld.z() + digital_filter_change_rate * posNew.z()
    );
    dst.setOrigin(pos);

    // I modified this in order to avoid singularities in angles
    tf::Quaternion ornOld = dst.getRotation();
    tf::Quaternion ornNew = src.getRotation();
    tf::Quaternion orn;

    // Interpolation between two quaternions.
    orn = ornOld.slerp(ornNew,digital_filter_change_rate);
    dst.setRotation(orn);

    return dst;
}
