#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

#define ROOMBA_CMD "/cmd_vel_mux/input/teleop"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  
  ros::Publisher chatter_pub1 = n.advertise<geometry_msgs::Twist>(ROOMBA_CMD, 1);

  ros::Rate loop_rate(5);
  int count = 0;
  while (ros::ok())
  {

    geometry_msgs::Twist msg;

    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.z = 0;
    
    if (count < 50) {msg.angular.z  = 0.1;}
    else if (count < 100) {msg.angular.z = -0.1;}
    else if (count < 150) {msg.angular.z = 1;}
    else if (count < 200) {msg.angular.z = -1;}
    else if (count < 250) {msg.angular.z = 0.5;}
    else if (count < 300) {msg.angular.z = -0.5;}
    else if (count < 350) {msg.angular.z = 0.75;}
    else if (count < 400) {msg.angular.z = -0.75;}
    else if (count < 450) {msg.angular.z = 0.25;}
    else if (count < 500) {msg.angular.z = -0.25;}
    else {msg.angular.z = 0;}


    chatter_pub1.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }


  return 0;
}
