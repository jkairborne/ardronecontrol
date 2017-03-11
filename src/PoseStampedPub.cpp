#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("vrpn_client_node/Ardrone/pose", 5);
  ros::Publisher vector_pub = n.advertise<geometry_msgs::Vector3>("desired_pos", 5);

  ros::Rate loop_rate(0.5);
  int count = 0;
  while (ros::ok())
  {
    count++; 
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = 0.1;
    msg.pose.position.y = 0.5;
    msg.pose.position.z = 0.3;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;

    geometry_msgs::Vector3 des_pos;

    des_pos.x = count;
    des_pos.y = 0;
    des_pos.z = -count;

    chatter_pub.publish(msg);
    vector_pub.publish(des_pos);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
