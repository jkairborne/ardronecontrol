// This is meant to average out the last...1500 or so errors. 
// This equates to approx 15s, and should give us a rough idea 
// of the performance of the controller.

// Need to add something to find the setpoint, as well as the RMS error averaging...

#include "pid.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "OptiTools.h"

#include <dynamic_reconfigure/server.h>
#include <ardronecontrol/PIDsetConfig.h>

#include <stdio.h>
#include <math.h>

#define array_length 1500

// Here I use global subscriber, since I want to access the
// publisher in the function MsgCallback:
ros::Subscriber pose_subscriber;

double x_des = 1.0;
double y_des = 1.0;
double z_des = 0.4;
double x_RMS[array_length];
double y_RMS[array_length];
int k = 0;

void leftRotatebyOne(double arr[], int n, double new_val);
double sumArray(double arr[], int n);
void printArray(double arr[], int size);
void callback(ardronecontrol::PIDsetConfig &config, uint32_t level);
void MsgCallback(const geometry_msgs::PoseStamped msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ArdroneRMS");
    ros::NodeHandle n;

    dynamic_reconfigure::Server<ardronecontrol::PIDsetConfig> server;
    dynamic_reconfigure::Server<ardronecontrol::PIDsetConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    // Subscribe to the Ardrone data incoming from the OptiTrack
    pose_subscriber = n.subscribe("/vrpn_client_node/Ardrone/pose", 5, MsgCallback);
    

    ros::spin();


    return 0;
}



void leftRotatebyOne(double arr[], int n, double new_val)
{
      int i; 
      double temp;
      temp = arr[0];
      for (i = 0; i < n-1; i++)
         arr[i] = arr[i+1];
      arr[i] = new_val*new_val;
}

double sumArray(double arr[], int n)
{
    int i;
    double temp=0;
    for(i=0;i<n;i++){temp+=arr[i];}

    temp = sqrt((1/(double) n)*temp);
    return temp;
}

void printArray(double arr[], int size)
{
    int i;
    for(i = 0; i < size; i++)
        ROS_INFO(" %f ", arr[i]);
}


void callback(ardronecontrol::PIDsetConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request setpoint: %f", 
             config.set_x);
  x_des = config.set_x;
  y_des = config.set_y;
}

// Workhorse function. rectifies coordinate system, converts quaternion to rpy, 
// converts from world to body frame, applies PIDs to the channels, then
// outputs the message onto a "/cmd_vel" topic.
void MsgCallback(const geometry_msgs::PoseStamped msg)
{
    geometry_msgs::PoseStamped pose_fixt;
    geometry_msgs::Quaternion GMquat;

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
    delta_z = pose_fixt.pose.position.z-z_des;

    leftRotatebyOne(x_RMS,array_length,delta_x);
    leftRotatebyOne(y_RMS,array_length,delta_y);
    double current_xRMS;
    double current_yRMS;
    current_xRMS = sumArray(x_RMS,array_length);
    current_yRMS = sumArray(y_RMS,array_length);

    k+=1;
    if(k/100 == 1)
    {
        k-=100;
        ROS_INFO("RMSx = %.2f, delta_x = %.2f, RMSy = %.2f, delta_y = %.2f",current_xRMS,delta_x,current_yRMS,delta_y);
    }
}

