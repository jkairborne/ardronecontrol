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

//#define USEROOMBA_VEL
//#define USEROOMBA_ACC

#define ZDES 300 //desired height in cm

#define KPLAT 0.0005
#define KDLAT 0.00075

double saturate_bounds(double max, double min, double val);
void printnavdata(ardrone_autonomy::Navdata msg);
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
    double Kp_x = KPLAT;
    double Kd_x = KDLAT;
    double Ki_x = 0.0;
    double Kp_y = KPLAT;
    double Kd_y = KDLAT;
    double Ki_y = 0.0;
    double Kp_z = 0.01;
    double Kd_z = 0;
    double Ki_z = 0;
    double dt = 0.03333; //tag detection takes place onboard at 30Hz
    double Kp_psi = 0.01;
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
    double Kp_x_vel = 0.1;
    double Kd_x_vel = 0.0;
    double Ki_x_vel = 0.0;
    PID pidx_vel = PID(dt,1,-1,Kp_x_vel,Kd_x_vel,Ki_x_vel);
    double x_vel_component;
#endif
#ifdef USEROOMBA_ACC
    double Kp_x_acc = 0.1;
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
  // Call the mod_params function of the Pimpl class - this then calls the set_gains function in the PID class which actually changes the gains used for calculations
  pidx.mod_params(Kp_x,Kd_x,Ki_x);
  // Change the desired positions
  x_des = config.set_x;

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


// Save the new configuration to doubles
  Kp_y = config.Kp_y;
  Kd_y = config.Kd_y;
  Ki_y = config.Ki_y;
  // Call the mod_params function of the Pimpl class - this then calls the set_gains function in the PID class which actually changes the gains used for calculations
  pidy.mod_params(Kp_y, Kd_y,Ki_y);
  // Change the desired positions
  y_des = config.set_y;

std::cout << "new gains: kp,d,i x: " << Kp_x << " " << Kd_x  << " " << Ki_x  << " " << "kp,d,i y: " << Kp_y << " " << Kd_y << " " << Ki_y << '\n';
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
    printnavdata(msg);
    newtime = msg.header.stamp;
    dt_ros = newtime-oldtime;
    dt = dt_ros.toSec();

    // Create the output message to be published
    geometry_msgs::Twist pid_output;

    if(msg.tags_count ==0)
    {
        srcCmd.publish(cmdNotPBVS);
        pid_output.linear.x = 0;
        pid_output.linear.y = 0;
        pid_output.linear.z = 0;
        pid_output.angular.x = 0.1;
        pid_output.angular.y = 0.1;// Send a constant angular 0.1 in y - this has no effect other than to remove the "auto-hover" function in ardrone-autonomy
        pid_output.angular.z = 0;
        quad_twist.publish(pid_output);
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
    zpos0 = msg.tags_distance[0];
    //psi needs somewhat special treatment, because for ArDrone it gets reported in degrees, from 0 to 360
    //delta_psi = (180-msg.tags_orientation[0]); // original, "proper" orientation
    delta_psi = msg.tags_orientation[0]-90;
    if ((delta_psi)>180)
    {
        delta_psi = 180-delta_psi;
    }

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



    // Populate the output message
    //Note that we swap deltax and deltay because of coordinate system differences
    pid_output.linear.x = pidx.calculate(0,delta_y,dt);
    pid_output.linear.y = pidy.calculate(0,delta_x,dt);
    pid_output.linear.z = pidz.calculate(ZDES,zpos0,dt);
/*
    std::cout << "Y delta, x output: " << delta_y << "   " << pid_output.linear.x << '\n';
    std::cout << "X delta, y output: " << delta_x << "   " << pid_output.linear.y << '\n';
    std::cout << "Z delta, z output: " << (zpos0-ZDES) << "   " << pid_output.linear.z << '\n';
*/
    // Send a constant angular 0.1 in y - this has no effect other than to remove the "auto-hover" function in ardrone-autonomy
    pid_output.angular.y = 0.1;
    pid_output.angular.z = -pidpsi.calculate(0,delta_psi,dt);

    std::cout << "psi delta-180, psi output: " << delta_psi << "   " << pid_output.angular.z << '\n';

    oldtime = newtime;

#ifdef USEROOMBA_ACC
    pid_output.linear.x += x_acc_component;
#endif
#ifdef USEROOMBA_VEL
    pid_output.linear.x += x_vel_component;
#endif
    pid_output.linear.x = saturate_bounds(1,-1,pid_output.linear.x);
    // publish PID output:
    quad_twist.publish(pid_output);
}

void roombaCallback(const geometry_msgs::TwistStamped& velcmd)
{
    ros::Time newtime_r = velcmd.header.stamp;

    dt_ros_r = newtime_r - oldtime_r;
    double timediff_r = dt_ros_r.toSec();
    double newvel_r = velcmd.twist.linear.x;
#ifdef USEROOMBA_ACC
    double acc = (newvel_r-oldvel_r)/timediff_r;
    x_acc_component =pidx_acc.calculate(0,acc,timediff_r);
    oldvel_r = newvel_r;

//    std::cout << "line 240: x_acc_component = " << x_acc_component << " , dt_ros_r =" << timediff_r << '\n';
#endif
#ifdef USEROOMBA_VEL
    x_vel_component = pidx_vel.calculate(0,velcmd.twist.linear.x,timediff_r);
//        std::cout << "line 244: x_vel_component = " << x_vel_component << " , vel_cmd =" << velcmd.twist.linear.x<< '\n';
#endif
    oldtime_r = newtime_r;
}

double virtcam(double origImgPts[],double roll, double pitch)
{
 /*   Eigen::Matrix3d output;
    double camRoll, camPitch;
    camRoll = -pitch;
    camPitch = -roll;

    
    roll = msg.rotX*M_PI/180;
    pitch = msg.rotY*M_PI/180;
    yaw = msg.rotZ*M_PI/180;
    // Number 2 from May 29th 2017 log
    output(0,0) = cos(camPitch);
    output(1,0) = 0;
    output(0,1) = sin(camRoll)*sin(camPitch);
    output(1,1) = cos(camRoll);
    output(2,0) = -sin(camPitch);
    output(2,1) = cos(camPitch)*sin(camRoll);
    output(2,2) = cos(camPitch)*cos(camRoll);
    output(0,2) = cos(camRoll)*sin(camPitch);
    output(1,2) = -sin(camRoll);




//23    std::cout << "callback roll, pitch, yaw: " << roll << '\t' << pitch << '\t' << yaw << '\n';
    std::vector<double> abc(3);
    abc.resize(3);
//23    std::cout <<"\n Just before abc \n";
    abc = get_rpy();
    //getRotM();
    
    */
    return 0.0;
}

double saturate_bounds(double max, double min, double val)
{
//    std::cout << "in saturate bounds - max: " << max << " min: " << min << " val: " << val << '\n';
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
    quad_twist = n.advertise<geometry_msgs::Twist>("cmd_vel_PBVS", 1);
    srcCmd = n.advertise<std_msgs::Float64>("src_cmd",1);

    new_gains = n.advertise<geometry_msgs::TwistStamped>("gain_changer", 1);
cmdPBVS.data = 1;
cmdNotPBVS.data = 0;

    // These four lines set up the dynamic reconfigure server
    dynamic_reconfigure::Server<ardronecontrol::PIDsetConfig> server;
    dynamic_reconfigure::Server<ardronecontrol::PIDsetConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);


    oldtime = ros::Time::now();
    // Subscribe to the Ardrone data incoming from the OptiTrack
    ardrone_subscriber = n.subscribe("/ardrone/navdata", 1, MsgCallback);
#if defined(USEROOMBA_VEL) || defined(USEROOMBA_ACC)
    roomba_subscriber = n.subscribe("/roomba_vel_cmd",1,roombaCallback);
#endif
    ros::spin();


    return 0;
}

void printnavdata(ardrone_autonomy::Navdata msg)
{
    if(msg.tags_count>0)
    {
        std::cout << "x/y/z/psi: " << msg.tags_xc[0] << "/" << msg.tags_yc[0] << "/" << msg.tags_distance[0] << "/" << msg.tags_orientation[0] << "\n";
    }
}


