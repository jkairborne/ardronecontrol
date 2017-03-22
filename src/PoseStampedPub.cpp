#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("vrpn_client_node/Ardrone/pose", 5);

  ros::Rate loop_rate(1);
  int count =0;
  double time = 0;
  while (ros::ok())
  {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = 1;
    msg.pose.position.y = 1*cos(time);
    msg.pose.position.z = 1;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;
    ROS_INFO("angle is %.2f, z is %.2f, count %d",time*180/3.1415926,msg.pose.position.y, count);
            

    time += 0.05*3.1415926;
    count++;
    chatter_pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
