#include "tud_coop_uv/SetController.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include <unordered_map>

double height  = 2;
double velx    = 0;
double vely    = 0;
double angz    = 0;
std::unordered_map<std::string,uint> controller({{"joy",0},{"tracking",1},{"joint",2}});
bool buttonTakeoff   = false;
bool buttonLand   = false;
bool buttonEnable = false;
bool buttonControlJoy = false;
bool buttonControlTracking = false;
bool buttonControlJoint = false;

std_msgs::Empty empty;

ros::Subscriber cmd_vel_sub;
ros::Publisher  twist_pub, takeoff_pub, land_pub;
ros::ServiceClient set_controller_client;
tud_coop_uv::SetController srv;

ros::Time buttonTakeoff_pressed_instant;
ros::Duration delay_button(1.0);

void set_hover(void){
    geometry_msgs::Twist cmd_vel_out;
    cmd_vel_out.linear.x = 0;
    cmd_vel_out.linear.y = 0;
    cmd_vel_out.linear.z = 0;
    cmd_vel_out.angular.x = 0;
    cmd_vel_out.angular.y = 0;
    cmd_vel_out.angular.z = 0;
    twist_pub.publish(cmd_vel_out);
}



void joyCallback(const sensor_msgs::Joy& in)
{
    geometry_msgs::Twist out_twist;
    velx   = double(in.axes[1]);
    vely   = double(in.axes[0]);
    height = double(in.axes[2])*3;
    angz   = double(in.axes[3]);
    buttonEnable = bool(in.buttons[0]);
    buttonLand = bool(in.buttons[1]);
    buttonTakeoff = bool(in.buttons[2]);
    buttonControlJoy = bool(in.buttons[3]);
    buttonControlTracking = bool(in.buttons[4]);
    buttonControlJoint = bool(in.buttons[5]);

    if (buttonControlJoy || buttonControlTracking || buttonControlJoint)
    {
        if (buttonControlJoy){
            srv.request.controller = controller["joy"];
        }
        else if (buttonControlTracking){
            srv.request.controller = controller["tracking"];
        }
        else if(buttonControlJoint){
            srv.request.controller = controller["joint"];
        }

        //call service in node Merge
        if (set_controller_client.call(srv)){
            if (srv.response.result){
                ROS_INFO("Succesfully changed controller");
            }
            else{
                ROS_INFO("Problem changing controller, server replied false");
            }
        }
        else{
            ROS_INFO("Failed to call service /tud_coop_uv/set_controller");
        }
    }




    if (buttonTakeoff){
        if ((ros::Time::now() - buttonTakeoff_pressed_instant) > delay_button){
            buttonTakeoff_pressed_instant = ros::Time::now();
            takeoff_pub.publish(empty);
            ROS_INFO("Button Pressed");
        }
        else{
            ROS_INFO("Waiting delay)");
        }
    }
    if (buttonLand){
        land_pub.publish(empty);
    }

    out_twist.linear.x = velx;
    out_twist.linear.y = vely;
    out_twist.linear.z = height;
    out_twist.angular.z = angz;
    //To Disable auto hover


    if (buttonEnable){
        out_twist.angular.x = 1.0;
        out_twist.angular.y = 1.0;
    }
    else{
        out_twist.angular.x = 0.0;
        out_twist.angular.y = 0.0;
    }

    twist_pub.publish(out_twist);

}




int main(int argc,char* argv[])
{
    ros::init(argc, argv, "joy_control"); // Name of the node
    ros::NodeHandle nh;



    buttonTakeoff_pressed_instant = ros::Time::now();

    set_controller_client = nh.serviceClient<tud_coop_uv::SetController>("tud_coop_uv/set_controller");
    cmd_vel_sub = nh.subscribe("/joy",1,joyCallback);
    twist_pub   = nh.advertise<geometry_msgs::Twist>("/joy/cmd_vel", 1);
    takeoff_pub = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    land_pub    = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
    ros::Rate rate(10); // 10 hz

    while(nh.ok()){
        ros::spinOnce();
        rate.sleep();
        }

}

