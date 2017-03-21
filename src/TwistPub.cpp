#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  
  ros::Publisher chatter_pub1 = n.advertise<geometry_msgs::Twist>("chat1", 1);
  ros::Publisher chatter_pub2 = n.advertise<geometry_msgs::Twist>("chat2", 1);
  ros::Publisher chatter_pub3 = n.advertise<geometry_msgs::Twist>("chat3", 1);

  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    geometry_msgs::Twist msg1;
    geometry_msgs::Twist msg2;
    geometry_msgs::Twist msg3;
    msg1.linear.x = 1;
    msg2.linear.x = 1;
    msg2.linear.y = 1;
    msg3.linear.x = 1;
    msg3.linear.y = 1;
    msg3.linear.z = 1;

    chatter_pub1.publish(msg1);
    chatter_pub2.publish(msg2);
    chatter_pub3.publish(msg3);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
