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
//#define USEROOMBA_ACC
//#define AUTODESCENT
//#define TARGETLOST

#define ZDES 200 //desired height in cm

#define KPLAT 0.035
#define KDLAT 0.25

#define TARGETSCALE (12/(100*22.2))
#define FX 670
#define FY 670
#define X0 500//470
#define Y0 445//387.4


double saturate_bounds(double max, double min, double val);
void printnavdata(ardrone_autonomy::Navdata msg);
void virtcam(double origImgPts[], double camRoll, double camPitch);

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
    bool visibleInLast;
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
double z_des = 0.6;
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

#if defined(AUTODESCENT) || defined(TARGETLOST)
    ros::Time lastTransition;
    double timeSinceTransition;

#endif
#ifdef TARGETLOST
    float DR_Scale = 0.1;
    geometry_msgs::Vector3 lastKnownCoords,deadReckoning;
    geometry_msgs::Vector3 getLastVec(geometry_msgs::Vector3 inPts)
    {
        geometry_msgs::Vector3 res;

        res.x = DR_Scale*inPts.x/sqrt(inPts.x*inPts.x + inPts.y * inPts.y);
        res.y = DR_Scale*inPts.y/sqrt(inPts.x*inPts.x + inPts.y * inPts.y);

        return res;
    }

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
  pidy.mod_params(Kp_x,Kd_x,Ki_x);
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
  pidx_acc.mod_params(Kplinear_x_acc,Kd_x_acc,Ki_x_acc);
#endif

/*
// Save the new configuration to doubles
  Kp_y = config.Kp_y;
  Kd_y = config.Kd_y;
  Ki_y = config.Ki_y;
  // Call the mod_params function of the Pimpl class - this then calls the set_gains function in the PID class which actually changes the gains used for calculations
  pidy.mod_params(Kp_y, Kd_y,Ki_y);
  // Change the desired positions
  y_des = config.set_y;
  newgains.header.stamp = ros::Time::now();
  newgains.twist.linear.x = Kp_x;
  newgains.twist.linear.y = Kd_x;
  newgains.twist.linear.z = Ki_x;
  newgains.twist.angular.x = Kp_y;
  newgains.twist.angular.y = Kd_y;
  newgains.twist.angular.z = Ki_y;
*/
//std::cout << "new gains: kp,d,i x: " << Kp_x << " " << Kd_x  << " " << Ki_x  << " " << "kp,d,i y: " << Kp_y << " " << Kd_y << " " << Ki_y << '\n';
  // Create and publish the new gains to a TwistStamped method:
  geometry_msgs::TwistStamped newgains;

  newgains.header.stamp = ros::Time::now();
  newgains.twist.linear.x = Kp_x;
  newgains.twist.linear.y = Kd_x;
  newgains.twist.linear.z = Ki_x;
  newgains.twist.angular.x = Kp_x;
  newgains.twist.angular.y = Kd_x;
  newgains.twist.angular.z = Ki_x;
  new_gains.publish(newgains);
}


// Main function. rectifies coordinate system, converts quaternion to rpy, 
// converts from world to body frame, applies PIDs to the channels, then
// outputs the message onto a "/cmd_vel" topic.

void MsgCallback(const ardrone_autonomy::Navdata msg)
{
  //  printnavdata(msg);
    newtime = msg.header.stamp;
    dt_ros = newtime-oldtime;
    dt = dt_ros.toSec();
    //std::cout << "received NalastTransitionvdata message\n";

    // Create the output mess650.87age to be published
    geometry_msgs::Twist pid_output;
    pid_output.linear.x = 0;
    pid_output.linear.y = 0;
    pid_output.linear.z = 0;
    pid_output.angular.x = 0;
    pid_output.angular.y = 0;
    pid_output.angular.z = 0;
/*
    if(msg.tags_count ==0)
    {
        pid_output.linear.x = 0;
        pid_output.linear.y = 0;
        pid_output.linear.z = 0;
        pid_output.angular.x = 0;
        pid_output.angular.y = 0;// Send a constant angular 111 in y - this has no effect other than to remove the "auto-hover" function in ardrone-autonomy
        pid_output.angular.z = -1000;

        if(targetVisible) // this means target was seen in last frame but not in this one
        {
            srcCmd.publish(cmdNotPBVS);

#ifdef TARGETLOST
            deadReckoning = getLastVec(lastKnownCoords);
            lastTransition = ros::Time::now();
        }
        timeSinceTransition = (ros::Time::now() - lastTransition).toSec();
        //std::cout << "timeSinceTrans TARGETLOST " << timeSinceTransition << '\n';
        if(timeSinceTransition < 3.0)
        {
            pid_output.linear.x = deadReckoning.x;
            pid_output.linear.y = deadReckoning.y;
            pid_output.linear.z = 0.1;
            pid_output.angular.x = 0;
            pid_output.angular.y = 0;
            pid_output.angular.z = 1000; // Send a constant angular 1000 in angular z - this tells the position controller we're dead reckoning
            std::cout << "<3s since target lost: x: " << pid_output.linear.x << " y: " << pid_output.linear.y << '\n';
#endif
        }
        //std::cout << "target lost: x: " << pid_output.linear.x << " y: " << pid_output.linear.y << '\n';
        quad_twist.publish(pid_output);
        targetVisible = 0;

        return;
    }
    if(targetVisible == 0) // This means target was not visible in last frame, but is in this one.
    {
#if defined(AUTODESCENT)
        lastTransition = ros::Time::now();
#endif
        pidx.rst_integral();
        pidy.rst_integral();
        pidz.rst_integral();
        pidpsi.rst_integral();

        srcCmd.publish(cmdPBVS);
        dt = 0; //This will use the default time step specified when the PID was created - and prevents too large of one being used.
        targetVisible = 1;
    }

    double xtag, ytag, xpos0, ypos0,zpos0, delta_x,delta_y,delta_z,delta_psi;

    std::cout << '\n' << msg.tags_xc[0] << " " << msg.tags_yc[0] << "\n";
    xtag = (double) msg.tags_xc[0]; // necessary to avoid int/double errors
    ytag = (double) msg.tags_yc[0];

    xpos0 = (xtag-X0)/FX;
    ypos0 = (ytag-Y0)/FY;
    zpos0 = msg.tags_distance[0]*TARGETSCALE;// See sept 9th 2017 notes

//s somewhat special treatment, because for ArDrone it gets reported in degrees, from 0 to 360
    //delta_psi = (180-msg.tags_orientation[0]); // original, "proper" orientation
    delta_psi = msg.tags_orientation[0]-90;
    if ((delta_psi)>180)
    {
        delta_psi = 180-delta_psi;
    }
    double orig[2], virt[2];
    orig[0] = xpos0;
    orig[1] = ypos0;
    orig[2] = zpos0;
    std::cout << xtag << " " << ytag << "\n";
    //std::cout << "original: " << xpos0 << " " << ypos0 << " " << zpos0 << '\n';
    //virtcam(orig,0, 0);
    virtcam(orig,(-msg.rotY*M_PI/180), (-msg.rotX*M_PI/180));

    xpos0 = orig[0];
    ypos0 = orig[1];
    zpos0 = orig[2];
    //std::cout << "modified: " << xpos0 << " " << ypos0 <<" " << zpos0 << '\n';

    delta_x = ((xpos0 * zpos0)-0.047)/1.5997;
    delta_y = ((ypos0 * zpos0)+0.0439)/2.1789;

    std::cout << "deltax, deltay: " << delta_x << " " << delta_y << "\n";

    // Populate the output message
    //Note that we swap deltax and deltay because of coordinate system differences
    pid_output.linear.x = pidx.calculate(0,delta_y,dt);
    pid_output.linear.y = pidy.calculate(0,delta_x,dt);
    pid_output.linear.z = pidz.calculate(ZDES,zpos0,dt);

    //std::cout << "Y delta, x output: " << delta_y << "   " << pid_output.linear.x << '\n';
    //std::cout << "X delta, y output: " << delta_x << "   " << pid_output.linear.y << '\n';
   // std::cout << "Z delta, z output: " << (zpos0-ZDES) << "   " << pid_output.linear.z << '\n';

#ifdef AUTODESCENT
    timeSinceTransition = (ros::Time::now() - lastTransition).toSec();
    //std::cout << "timeSinceTrans AUTODESCENT " << timeSinceTransition << '\n';
    if(timeSinceTransition > 2.0)
    {
        pid_output.linear.z = -0.1;
//        pid_output.angular.x = 999;
//        pid_output.angular.y = -999;
    }
#endif


    // Send the delta y in angular x, and the delta x in angular y - this is for the optitrack control on the position_controller side.
    pid_output.angular.x = delta_y;
    pid_output.angular.y = delta_x;
    pid_output.angular.z = -pidpsi.calculate(0,delta_psi,dt);

#ifdef TARGETLOST
    lastKnownCoords.x = delta_x;
    lastKnownCoords.y = delta_y;
#endif
    oldtime = newtime;
*/
#ifdef USEROOMBA_ACC
    pid_output.linear.y += x_acc_component; // this assumes the rover is moving along y axis
#endif
#ifdef USEROOMBA_VEL
    pid_output.linear.y += x_vel_component;// this assumes the rover is moving along y axis
#endif
    // Saturate then publish PID output:
    pid_output.linear.x = saturate_bounds(1,-1,pid_output.linear.x);
    pid_output.linear.y = saturate_bounds(1,-1,pid_output.linear.y);
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

void virtcam(double origImgPts[],double camRoll, double camPitch)
{
    double u0 = origImgPts[0];
    double v0 = origImgPts[1];
    double z_est = origImgPts[2];

    double x = u0*z_est;
    double y = v0*z_est;

    double x_v = cos(camPitch)*x + sin(camRoll)*sin(camPitch)*y + cos(camRoll)*sin(camPitch)*z_est;
    double y_v = cos(camRoll)*y-sin(camRoll)*z_est;
    double z_v = -sin(camPitch)*x+cos(camPitch)*sin(camRoll)*y+cos(camPitch)*cos(camRoll)*z_est;

    origImgPts[0] = x_v/z_v;
    origImgPts[1] = y_v/z_v;
    origImgPts[2] = z_v;

    //std::cout << "u0,v0: " << u0 << " " << v0 << " x,y " << x << " " << y << " x_v, y_v: " << x_v << " " << y_v << " output: " << origImgPts[0] << " " << origImgPts[1] << '\n';

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

std::cout << "In beginning";

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


