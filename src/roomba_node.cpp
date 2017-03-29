#include "ros/ros.h"
#include "ros/time.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include <math.h>
#include <sstream>

#define ROOMBA_CMD "/cmd_vel_mux/input/teleop"
#define ROOMBA_VALS "/roomba_values"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roomba_node");

  ros::NodeHandle n;

  // Create a Publisher to command the roomba
  ros::Publisher cmd_to_roomba = n.advertise<geometry_msgs::Twist>(ROOMBA_CMD, 3);
  // Create a Vector3 containing position (null in this case), angular rate and angular acceleration
  ros::Publisher R_values = n.advertise<geometry_msgs::Vector3>(ROOMBA_VALS, 3);

  ros::Rate loop_rate(100);

  int count = 0;
  ros::Time oldtime, newtime;
  ros::Duration timeDiff;
  float oldvel = 0.0;

  while (ros::ok())
  {
    geometry_msgs::Twist cmdvel;
    geometry_msgs::Vector3 ang_vals;
    std_msgs::Float64 xvel_control;
    float cmdx = 0.0;
    float cmdz = 0.5*sin(count/500.0);

	// Populate the Twist message to be sent to the Roomba itself
    cmdvel.linear.x = cmdx;
    cmdvel.angular.z = cmdz;

	// Define the message to be sent to the PID for the velocity portion
    xvel_control.data = cmdx;

	// Define the message to be published to the acceleration command (requires a timestamp)
    ang_vals.x = 0.0;
    ang_vals.y = cmdz;
    newtime = ros::Time::now();

    timeDiff = oldtime - newtime;
    ang_vals.z = (oldvel-cmdz)/timeDiff.toSec();

	//Publish all three messages
    cmd_to_roomba.publish(cmdvel);
    R_values.publish(ang_vals);
  
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
    oldtime = newtime;
    oldvel = cmdz;
  }

  return 0;
}
