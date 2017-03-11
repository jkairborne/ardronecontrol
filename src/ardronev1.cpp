#include "pid.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "OptiTools.h"

#include <stdio.h>
#include <math.h>

// Here I use global publisher and subscriber, since I want to access the
// publisher in the function MsgCallback:
ros::Publisher quad_twist;
ros::Subscriber pose_subscriber;
ros::Time time1, time2, time3;
ros::Duration diff1, diff2;

// Main function. rectifies coordinate system, converts quaternion to rpy, 
// converts from world to body frame, applies PIDs to the channels, then
// outputs the message onto a "/cmd_vel" topic.
void MsgCallback(const geometry_msgs::PoseStamped msg)
{
    geometry_msgs::PoseStamped pose_fixt;
    geometry_msgs::Quaternion GMquat;
    double x_des, y_des, z_des, Kp_xy,Kp_z,Kd_xy,Kd_z,Ki_xy,Ki_z;
    // Define controller setpoints
    x_des = 1.0;
    y_des = 0.65;
    z_des = 0.25;
    // Define controller gains
    Kp_xy = 1.0;
    Kd_xy = 0;
    Ki_xy = 0;
    Kp_z = 1.0;
    Kd_z = 0;
    Ki_z = 0;

    // Here the Opti_Rect function is defined in OptiTools.h, and simply adjusts the coordinate system to be the one that we are used to working with.
    pose_fixt = Opti_Rect(msg);
    GMquat = pose_fixt.pose.orientation;

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(GMquat, quat);
//    quat = tf::Quaternion(quattemp.x(),-quattemp.z(),quattemp.y(),quattemp.w());

    // the tf::Quaternion has a method to acess roll pitch and yaw, which we use here
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // Calculate delta_x and delta_y in the body-fixed frame.
    double delta_x,delta_y,delta_z;
    delta_x = cos(yaw)*(pose_fixt.pose.position.x-x_des) + sin(yaw)*(pose_fixt.pose.position.y-y_des);
    delta_y = -sin(yaw)*(pose_fixt.pose.position.x-x_des) + cos(yaw)*(pose_fixt.pose.position.y-y_des);
    delta_z = pose_fixt.pose.position.z-z_des;

    // Create the output message to be published
    geometry_msgs::Twist pid_output;

    // Create the PID class instances for x, y, and z:
    PID pidx = PID(0.1,1,-1,Kp_xy,Kd_xy,Ki_xy);
    PID pidy = PID(0.1,1,-1,Kp_xy,Kd_xy,Ki_xy);
    PID pidz = PID(0.1,1,-1,Kp_z,Kd_z,Ki_z);
    // Populate the output message
    pid_output.linear.x = pidx.calculate(0,delta_x);
    pid_output.linear.y = pidy.calculate(0,delta_y);
    pid_output.linear.z = pidz.calculate(0,delta_z);

    // publish PID output:
    quad_twist.publish(pid_output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ArdronePID");
    ros::NodeHandle n;
    // Advertise the cmd vel node
    quad_twist = n.advertise<geometry_msgs::Twist>("cmd_vel_opti", 5);
    // Subscribe to the Ardrone data incoming from the OptiTrack
    pose_subscriber = n.subscribe("/vrpn_client_node/Ardrone/pose", 5, MsgCallback);

    // check for incoming quaternions untill ctrl+c is pressed
    ROS_INFO("waiting for quaternion");

    ros::spin();


    return 0;
}
