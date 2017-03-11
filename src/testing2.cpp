// function example
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include "subscriber.h"
#include "OptiTools.h"
#include "ros/ros.h"

using namespace std;


geometry_msgs::PoseStamped firefly (geometry_msgs::PoseStamped msg1)
{
  geometry_msgs::PoseStamped msg2;
  msg2.pose.position.x = msg1.pose.position.x +1;
  return msg2;
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "testing2");
    ros::NodeHandle n;
  geometry_msgs::PoseStamped x, y;
  Subscriber sub1("/Ardrone/pose");
//    Subscriber sub1 = Subscriber("/Ardrone/pose");
    x = sub1.GetPose();
  y = firefly(x);
  cout << "The result is "  << x.pose.position.x << " then " << y.pose.position.x;
  ROS_INFO("The result is %.2f", x.pose.position.x);
  ros::spin();
  return 0;
}
