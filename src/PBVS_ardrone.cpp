#include "pid.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "OptiTools.h"
#include "ardrone_autonomy/Navdata.h"

#include <dynamic_reconfigure/server.h>
#include <ardronecontrol/PIDsetConfig.h>


#include <stdio.h>
#include <math.h>

//#define USEOPTI
//#define USEVICON

#define USEROOMBA_VEL
#define USEROOMBA_ACC

double saturate_bounds(double max, double min, double val);

// Here I use global publisher and subscriber, since I want to access the
// publisher in the function MsgCallback:
ros::Publisher quad_twist;
ros::Publisher srcCmd;
ros::Publisher new_gains;
ros::Subscriber ardrone_subscriber;
#if defined(USEROOMBA_VEL) || defined(USEROOMBA_ACC)
ros::Subscriber roomba_subscriber;
#endif
ros::Time newtime, oldtime;
ros::Duration dt_ros;
bool targetVisible;
std_msgs::Float64 cmdNotPBVS, cmdPBVS;

double oldvel_r;
ros::Time newtime_r, oldtime_r;
ros::Duration dt_ros_r;

    // Define controller gains
    double Kp_x = 0.1;
    double Kd_x = 0.0;
    double Ki_x = 0.0;
    double Kp_y = 0.1;
    double Kd_y = 0.2;
    double Ki_y = 0.0;
    double Kp_z = 0.7;
    double Kd_z = 0;
    double Ki_z = 0;
    double dt = 0.03333; //tag detection takes place onboard at 30Hz
    double Kp_psi = 0.7;
    double Kd_psi = 0.0;
    double Ki_psi = 0.0;
    //
// Define controller setpoints, in case there is no subscriber to callback
double x_des = 0.0;
double y_des = 0.0;
double z_des = 0.8;
    // Create the PID class instances for x, y, and z:
    PID pidx = PID(dt,1,-1,Kp_x,Kd_x,Ki_x);
    PID pidy = PID(dt,1,-1,Kp_y,Kd_y,Ki_y);
    PID pidz = PID(dt,1,-1,Kp_z,Kd_z,Ki_z);
    PID pidpsi = PID(dt,1,-1,Kp_psi,Kd_psi,Ki_psi);

#ifdef USEROOMBA_VEL
    double Kp_x_vel = 0.2;
    double Kd_x_vel = 0.0;
    double Ki_x_vel = 0.0;
    PID pidx_vel = PID(dt,1,-1,Kp_x_vel,Kd_x_vel,Ki_x_vel);
    double x_vel_component;
#endif
#ifdef USEROOMBA_ACC
    double Kp_x_acc = 0.2;
    double Kd_x_acc = 0.0;
    double Ki_x_acc = 0.0;
    PID pidx_acc = PID(dt,1,-1,Kp_x_acc,Kd_x_acc,Ki_x_acc);
    double x_acc_component;
#endif




    // This is the callback from the parameter server
void callback(ardronecontrol::PIDsetConfig &config, uint32_t level) {
//  ROS_INFO("Reconfigure Request: %f %f", 
//             config.Kp_x,config.set_x);

// Save the new configuration to doubles
  Kp_x = config.Kp_x;
  Kd_x = config.Kd_x;
  Ki_x = config.Ki_x;
#ifdef USEROOMBA_VEL
  Kp_x_vel = config.Kp_x_vel;
  Kd_x_vel = config.Kd_x_vel;
  Ki_x_vel = config.Ki_x_vel;
  pidx_vel.mod_params(Kp_x_vel,Kd_x_vel,Ki_x_vel);
#endif
#ifdef USEROOMBA_ACC
  Kp_x_acc = config.Kp_x_acc;
  Kd_x_acc = config.Kd_x_acc;
  Ki_x_acc = config.Ki_x_acc;
  pidx_acc.mod_params(Kp_x_acc,Kd_x_acc,Ki_x_acc);
#endif
  // Call the mod_params function of the Pimpl class - this then calls the set_gains function in the PID class which actually changes the gains used for calculations
  pidx.mod_params(Kp_x, Kd_x,Ki_x);
  // Change the desired positions
  x_des = config.set_x;

// Save the new configuration to doubles
  Kp_y = config.Kp_y;
  Kd_y = config.Kd_y;
  Ki_y = config.Ki_y;
  // Call the mod_params function of the Pimpl class - this then calls the set_gains function in the PID class which actually changes the gains used for calculations
  pidx.mod_params(Kp_y, Kd_y,Ki_y);
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

void MsgCallback(const ardrone_autonomy::Navdata msg)
{
    newtime = msg.header.stamp;
    dt_ros = newtime-oldtime;
    dt = dt_ros.toSec();

    if(msg.tags_count ==0)
    {
        srcCmd.publish(cmdNotPBVS);
        targetVisible = 0;
        return;
    }
    if(targetVisible == 0)
    {
        pidx.rst_integral();
        pidy.rst_integral();
        pidz.rst_integral();
        pidpsi.rst_integral();

        srcCmd.publish(cmdPBVS);
        dt = 0; //This will use the default time step specified when the PID was created - and prevents too large of one being used.
        targetVisible = 1;

    }

    // Assign new time into newtime global variable



    double xpos0, ypos0,zpos0, delta_x,delta_y,delta_z,delta_psi;

    xpos0 = (msg.tags_xc[0]-500.0)/878.41;
    ypos0 = (msg.tags_yc[0]-500.0)/917.19;
    delta_psi = msg.tags_orientation[0];
    zpos0 = msg.tags_distance[0];
 //   delta_z = zpos0 - 1.5; //height of 1.5m...

    delta_x = xpos0 * zpos0;
    delta_y = ypos0 * zpos0;
/*
    double rotx, roty, rotz;
    rotx = msg.rotX*M_PI/180.0;
    roty = msg.rotY*M_PI/180.0;
    rotz = msg.rotZ*M_PI/180.0;
    double derotx, deroty, derotz, xprime, yprime;

    xprime = msg.tags_distance[0]*sin(rotx - atan((msg.tags_xc[0]-500.0)/878.41)); // the 878.41 is the focal length in the x direction in units of pixels
    yprime = msg.tags_distance[0]*sin(roty - atan((msg.tags_yc[0]-500.0)/917.19)); // the 878.41 is the focal length in the x direction in units of pixels
*/

    // Create the output message to be published
    geometry_msgs::Twist pid_output;

    // Populate the output message
    pid_output.linear.x = pidx.calculate(0,delta_x,dt);
    pid_output.linear.y = pidy.calculate(0,delta_y,dt);
    pid_output.linear.z = pidz.calculate(1.5,zpos0,dt);
    // Send a constant angular 0.1 in y - this has no effect other than to remove the "auto-hover" function in ardrone-autonomy
    pid_output.angular.y = 0.1;
    pid_output.angular.z = pidpsi.calculate(0,delta_psi,dt);

    oldtime = newtime;

    std::cout << "Line 208 - value of pid output x: " << pid_output.linear.x << '\n';

#ifdef USEROOMBA_ACC
    pid_output.linear.x += x_acc_component;
    std::cout << "Line 212 - roomba accel - value of pid output x: " << pid_output.linear.x << '\n';
#endif
#ifdef USEROOMBA_VEL
    pid_output.linear.x += x_vel_component;
            std::cout << "Line 215 - roomba vel - value of pid output x: " << pid_output.linear.x << '\n';
#endif
    pid_output.linear.x = saturate_bounds(1,-1,pid_output.linear.x);
        std::cout << "Line 219 - after sat bounds - value of pid output x: " << pid_output.linear.x << '\n';
    // publish PID output:
    quad_twist.publish(pid_output);
}

double callbackacc(double newvel_r, double newdt)
{
    double acc = (newvel_r-oldvel_r)/newdt;

    double ret_val = pidx_acc.calculate(0,acc,newdt);
    std::cout << "line 231: newvel: " << newvel_r << " oldvel: " << oldvel_r << " dt: " << newdt << '\n';

    oldvel_r = newvel_r;

    return ret_val;
} //end callbackacc


void roombaCallback(const geometry_msgs::TwistStamped& velcmd)
{
    ros::Time newtime_r = velcmd.header.stamp;

    dt_ros_r = newtime_r - oldtime_r;
    double timediff_r = dt_ros_r.toSec();
#ifdef USEROOMBA_ACC
    x_acc_component =callbackacc(velcmd.twist.linear.x,timediff_r);
    std::cout << "line 240: x_acc_component = " << x_acc_component << " , dt_ros_r =" << timediff_r << '\n';
#endif
#ifdef USEROOMBA_VEL
    x_vel_component = pidx_vel.calculate(0,velcmd.twist.linear.x,timediff_r);
        std::cout << "line 244: x_vel_component = " << x_vel_component << " , vel_cmd =" << velcmd.twist.linear.x<< '\n';
#endif
    oldtime_r = newtime_r;

}

double saturate_bounds(double max, double min, double val)
{
    std::cout << "in saturate bounds - max: " << max << " min: " << min << " val: " << val << '\n';
    if(max < min)
    {
        std::cout << "Your Min is greater than your max in saturate_bounds fct!";
        return -1;
    }
    if(val>max){return max;}
    else if (val<min) {return min;}
    else {return val;}
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "ArdronePID");
    ros::NodeHandle n;

    // Advertise the cmd vel node
    quad_twist = n.advertise<geometry_msgs::Twist>("cmd_vel_opti", 1);
    srcCmd = n.advertise<std_msgs::Float64>("src_cmd",1);

    new_gains = n.advertise<geometry_msgs::TwistStamped>("gain_changer", 1);
cmdPBVS.data = 1;
cmdNotPBVS.data = 0;

    // These four lines set up the dynamic reconfigure server
/*    dynamic_reconfigure::Server<ardronecontrol::PIDsetConfig> server;
    dynamic_reconfigure::Server<ardronecontrol::PIDsetConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
*/

    oldtime = ros::Time::now();
    // Subscribe to the Ardrone data incoming from the OptiTrack
    ardrone_subscriber = n.subscribe("/ardrone/navdata", 1, MsgCallback);
#if defined(USEROOMBA_VEL) || defined(USEROOMBA_ACC)
    roomba_subscriber = n.subscribe("/roomba_vel_cmd",1,roombaCallback);
#endif
    ros::spin();


    return 0;
}
