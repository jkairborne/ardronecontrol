#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "ardrone_autonomy/Navdata.h"

//#define POSESTAMPED
#define TWISTSTAMPED

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Time startTime = ros::Time::now();


    ros::Publisher ardrone_pub = n.advertise<ardrone_autonomy::Navdata>("vrpn_client_node/ardrone/pose", 1);
    ardrone_autonomy::Navdata ar_msg;

#ifdef POSESTAMPED
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("vrpn_client_node/ardrone/pose", 1);
#endif
#ifdef TWISTSTAMPED
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::TwistStamped>("roomba_vel_cmd", 1);
#endif
  ros::Rate loop_rate(10);
  int count =0;


  while (ros::ok())
  {
#ifdef POSESTAMPED
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = 0.5;
    msg.pose.position.y = 0.5;
    msg.pose.position.z = 0.3;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;
#endif
#ifdef TWISTSTAMPED
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = 0.5*sin(count/100.0);
    msg.twist.linear.y = 0.5;
    msg.twist.linear.z = 0.3;
    msg.twist.angular.x = 0;
    msg.twist.angular.y = 0;
    msg.twist.angular.z = 0;
    std::cout << "msg time: " << (ros::Time::now()-startTime).toSec() << " \t x: " << msg.twist.linear.x <<"\n";
#endif
    count++;
    chatter_pub.publish(msg);
    ardrone_pub.publish(ar_msg);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
