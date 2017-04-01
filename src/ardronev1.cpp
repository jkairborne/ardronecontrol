#include "pid.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "OptiTools.h"

#include <dynamic_reconfigure/server.h>
#include <ardronecontrol/PIDsetConfig.h>

#include <stdio.h>
#include <math.h>

// Here I use global publisher and subscriber, since I want to access the
// publisher in the function MsgCallback:
ros::Publisher quad_twist;
ros::Publisher new_gains;
ros::Subscriber pose_subscriber;
ros::Time newtime, oldtime;
ros::Duration dt_ros;

    // Define controller gains
    double Kp_x = 0.4;
    double Kd_x = 0.8;
    double Ki_x = 0.0;
    double Kp_y = 0.4;
    double Kd_y = 0.8;
    double Ki_y = 0.0;
    double Kp_z = 0.7;
    double Kd_z = 0;
    double Ki_z = 0;
    double dt = 0.01;
    //
// Define controller setpoints, in case there is no subscriber to callback
double x_des = 1.0;
double y_des = 1.0;
double z_des = 0.6;
    // Create the PID class instances for x, y, and z:
    PID pidx = PID(0.01,1,-1,Kp_x,Kd_x,Ki_x);
    PID pidy = PID(0.01,1,-1,Kp_y,Kd_y,Ki_y);
    PID pidz = PID(0.01,1,-1,Kp_z,Kd_z,Ki_z);

    // This is the callback from the parameter server
void callback(ardronecontrol::PIDsetConfig &config, uint32_t level) {
//  ROS_INFO("Reconfigure Request: %f %f", 
//             config.Kp_x,config.set_x);

// Save the new configuration to doubles
  Kp_x = config.Kp_x;
  Kd_x = config.Kd_x;
  Ki_x = config.Ki_x;
  // Call the mod_params function of the Pimpl class - this then calls the set_gains function in the PID class which actually changes the gains used for calculations
  pidx.mod_params(Kp_x, Kd_x,Ki_x);
  // Change the desired positions
  x_des = config.set_x;

// Save the new configuration to doubles
  Kp_y = config.Kp_y;
  Kd_y = config.Kd_y;
  Ki_y = config.Ki_y;
  // Call the mod_params function of the Pimpl class - this then calls the set_gains function in the PID class which actually changes the gains used for calculations
  pidy.mod_params(Kp_y, Kd_y,Ki_y);
  // Change the desired positions
  y_des = config.set_y;


  // Create and publish the new gains to a TwistStamped method:
  geometry_msgs::TwistStamped newgains;
  newgains.header.stamp = ros::Time::now();
  newgains.twist.linear.x = Kp_x;
  newgains.twist.linear.y = Kd_x;
  newgains.twist.linear.z = Ki_x;
  newgains.twist.angular.x = Kp_y;
  newgains.twist.angular.y = Kd_y;
  newgains.twist.angular.z = Ki_y;

  new_gains.publish(newgains);
}


// Main function. rectifies coordinate system, converts quaternion to rpy, 
// converts from world to body frame, applies PIDs to the channels, then
// outputs the message onto a "/cmd_vel" topic.
void MsgCallback(const geometry_msgs::PoseStamped msg)
{
    geometry_msgs::PoseStamped pose_fixt;
    geometry_msgs::Quaternion GMquat;
    

    // Need to have all three gains have the same sign
    if ( !((Kp_x<=0. && Ki_x<=0. && Kd_x<=0.) || (Kp_x>=0. && Ki_x>=0. && Kd_x>=0.) || (Kp_y<=0. && Ki_y<=0. && Kd_y<=0.) || (Kp_y>=0. && Ki_y>=0. && Kd_y>=0.) || (Kp_z<=0. && Ki_z<=0. && Kd_z<=0.) || (Kp_z>=0. && Ki_z>=0. && Kd_z>=0.)) )
    {
        ROS_WARN("All three gains (Kp, Ki, Kd) should have the same sign for stability.");
    }

    // Assign new time into newtime global variable
    newtime = msg.header.stamp;
    dt_ros = newtime-oldtime;
    dt = dt_ros.toSec();

    // Here the Opti_Rect function is defined in OptiTools.h, and simply adjusts the coordinate system to be the one that we are used to working with.
    pose_fixt = Opti_Rect_quat(msg);
    GMquat = pose_fixt.pose.orientation;

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(GMquat, quat);

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

    // Populate the output message
    pid_output.linear.x = pidx.calculate(0,delta_x, dt);
    pid_output.linear.y = pidy.calculate(0,delta_y, dt);
    pid_output.linear.z = pidz.calculate(0,delta_z, dt);
    // Send a constant angular 0.1 in y - this has no effect other than to remove the "auto-hover" function in ardrone-autonomy
    pid_output.angular.y = 0.1;

    // Re-assign the times
    oldtime = newtime;

    // publish PID output:
    quad_twist.publish(pid_output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ArdronePID");
    ros::NodeHandle n;

    // Advertise the cmd vel node
    quad_twist = n.advertise<geometry_msgs::Twist>("cmd_vel_opti", 5);
    new_gains = n.advertise<geometry_msgs::TwistStamped>("gain_changer", 5);

    // These four lines set up the dynamic reconfigure server
    dynamic_reconfigure::Server<ardronecontrol::PIDsetConfig> server;
    dynamic_reconfigure::Server<ardronecontrol::PIDsetConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    oldtime = ros::Time::now();
    // Subscribe to the Ardrone data incoming from the OptiTrack
    pose_subscriber = n.subscribe("/vrpn_client_node/Ardrone/pose", 5, MsgCallback);


    ros::spin();


    return 0;
}
