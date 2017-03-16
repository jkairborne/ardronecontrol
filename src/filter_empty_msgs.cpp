#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"

ros::Publisher PassedThrough;

ros::Subscriber sub;

void chatterCallback(const geometry_msgs::Twist msg)
{
ROS_INFO("I heard: [%f, %f, %f]", msg.linear.x,msg.linear.y,msg.linear.z);

		if (msg.linear.x != 0 or msg.linear.y != 0 or msg.linear.z != 0 or msg.angular.z !=0)
			{
			PassedThrough.publish(msg);
			}
   	ros::Rate loop_rate(100);
	ros::spinOnce();
	loop_rate.sleep(); 
  }




int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter_node");

  ros::NodeHandle n;

  PassedThrough = n.advertise<geometry_msgs::Twist>("/key_filtered",1000);
  sub = n.subscribe("/key_output", 1000, chatterCallback);

  ros::spin();

  return 0;
}
