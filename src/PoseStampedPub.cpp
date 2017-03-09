#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("vrpn_client_node/Ardrone/pose", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = 0.1;
    msg.pose.position.y = 0.5;
    msg.pose.position.z = 2;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = -0.7071;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 0.7071;


    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
