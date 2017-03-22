#include "pid.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "OptiTools.h"
#include "std_msgs/Empty.h"

#include <stdio.h>
#include <math.h>

// Here I use global publisher and subscriber, since I want to access the
// publisher in the function MsgCallback:
ros::Publisher quad_twist;
ros::Publisher land_ardrone;
ros::Subscriber pose_subscriber;
ros::Time newtime, oldtime;
ros::Duration dt_ros;
ros::Time t_in_ge;
ros::Duration dt_ge;
 
    // Define controller gains
    double Kp_x = 0.3;
    double Kd_x = 0.6;
    double Ki_x = 0.0;
    double Kp_y = 0.3;
    double Kd_y = 0.6;
    double Ki_y = 0.0;
    double Kp_z = 0.8;
    double Kd_z = 0.4;
    double Ki_z = 0;
    double dt = 0.01;
    //
    // Create the PID class instances for x, y, and z:
    PID pidx = PID(0.01,1,-1,Kp_x,Kd_x,Ki_x);
    PID pidy = PID(0.01,1,-1,Kp_y,Kd_y,Ki_y);
    PID pidz = PID(0.01,1,-1,Kp_z,Kd_z,Ki_z);

    bool on_ground = 1;
    bool in_ge = 0;

// Main function. rectifies coordinate system, converts quaternion to rpy, 
// converts from world to body frame, applies PIDs to the channels, then
// outputs the message onto a "/cmd_vel" topic.
void MsgCallback(const geometry_msgs::PoseStamped msg)
{
    geometry_msgs::PoseStamped pose_fixt;
    geometry_msgs::Quaternion GMquat;
    

//Need to have all three gains have the same sign
//if ( !((Kp_xy<=0. && Ki_xy<=0. && Kd_xy<=0.) || (Kp_xy>=0. && Ki_xy>=0. && Kd_xy>=0.) || (Kp_z<=0. && Ki_z<=0. && Kd_z<=0.) || (Kp_z>=0. && Ki_z>=0. && Kd_z>=0.)) )
//{
//   ROS_WARN("All three gains (Kp, Ki, Kd) should have the same sign for stability.");
//}


// Define controller setpoints, in case there is no subscriber to callback
double x_des = 1.0;
double y_des = 1.0;
//double z_des = 0.4;

    // Assign new time into newtime global variable
    newtime = msg.header.stamp;
    dt_ros = newtime-oldtime;
    dt = dt_ros.toSec();

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
//    ROS_INFO("Roll = %.2f, Pitch = %.2f, Yaw = %.2f",roll*180/3.1415926,pitch*180/3.1415926,yaw*180/3.1415926);
    


    // Calculate delta_x and delta_y in the body-fixed frame.
    double delta_x,delta_y,delta_z;
    delta_x = cos(yaw)*(pose_fixt.pose.position.x-x_des) + sin(yaw)*(pose_fixt.pose.position.y-y_des);
    delta_y = -sin(yaw)*(pose_fixt.pose.position.x-x_des) + cos(yaw)*(pose_fixt.pose.position.y-y_des);
//NOT REQUIRED FOR SIMON'S LANDING    delta_z = pose_fixt.pose.position.z-z_des;

//   ROS_INFO("dt = %.2f, delta_x = %.2f, delta_y = %.2f",dt,delta_x,delta_y);

    // Create the output message to be published
    geometry_msgs::Twist pid_output;

    // Populate the output message
    pid_output.linear.x = pidx.calculate(0,delta_x);
    pid_output.linear.y = pidy.calculate(0,delta_y);
    // Send a constant angular 0.1 in y - this has no effect other than to remove the "auto-hover" function in ardrone-autonomy
    pid_output.angular.y = 0.1;

    //Section for Simon's landing controller:
    double z_act = pose_fixt.pose.position.z;
    // Parameters to be modified
    double tau = 0.5;
    double zdot_initial = -0.7;
    double zdot_touchdown = -0.07;


    double z_flare = -zdot_initial*0.7*tau;
    double z_land = 0.03;
    
    double z_ge = 0.15;


    if (z_act < z_land && !on_ground)
    {
        std_msgs::Empty landrone;
        land_ardrone.publish(landrone);
    }

    if(z_act>z_flare)
    {
        pid_output.linear.z = zdot_initial;
        on_ground = 0;
    }
    else
    {
        pid_output.linear.z = std::min(-z_act/(0.7*tau),zdot_touchdown);
    }


    if(z_act<z_ge && !in_ge)
    {
        t_in_ge = ros::Time::now();
        in_ge = 1;
        ROS_INFO("in 1");
    }
    else if (z_act>z_ge && ((ros::Time::now() - t_in_ge).toSec() > 1.0))
    {
        in_ge = 0;
        ROS_INFO("in 3");
    }
    else if(in_ge)
    {
        dt_ge = ros::Time::now() - t_in_ge;
        pid_output.linear.z = pid_output.linear.z - 0.1*dt_ge.toSec();
        ROS_INFO("in 2");
    }



    ROS_INFO("pid out = %.2f, z_act = %.2f, -z/tau = %.2f, in_ge = %d, pid_add = %.2f, t_in_ge = %.2f, dt_ge = %.2f, t = %.2f",pid_output.linear.z, z_act, -z_act/tau, in_ge, pid_output.linear.z - 0.1*dt_ge.toSec(), t_in_ge.toSec(), dt_ge.toSec(), (ros::Time::now() - t_in_ge).toSec() );

        // Re-assign the times
    oldtime = newtime;

    // publish PID output:
    quad_twist.publish(pid_output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ArdronePID");
    ros::NodeHandle n;
    oldtime = ros::Time::now();

    // Advertise the cmd vel node
    quad_twist = n.advertise<geometry_msgs::Twist>("cmd_vel_opti", 5);
    land_ardrone = n.advertise<std_msgs::Empty>("ardrone/land",5);
    // Subscribe to the Ardrone data incoming from the OptiTrack
    pose_subscriber = n.subscribe("/vrpn_client_node/Ardrone/pose", 5, MsgCallback);


    ros::spin();


    return 0;
}
