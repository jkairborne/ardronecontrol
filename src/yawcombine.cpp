#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

#define OPTI_TOPIC "/cmd_vel_opti1"
#define YAW_TOPIC "/visual_yaw"
#define OUTPUT_TOPIC "/cmd_vel_opti"

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::Twist>(OUTPUT_TOPIC, 1);

    //Topic you want to subscribe
    yawsub_ = n_.subscribe(YAW_TOPIC, 1, &SubscribeAndPublish::yawcallback, this);
    optisub_ = n_.subscribe(OPTI_TOPIC, 1, &SubscribeAndPublish::opticallback, this);
  }

  void yawcallback(const std_msgs::Float64& input)
  {
    output.angular.z = input.data;
    pub_.publish(output);
  }

  void opticallback(const geometry_msgs::Twist& input)
  {
    output.linear.x = input.linear.x;
    output.linear.y = input.linear.y;
    output.linear.z = input.linear.z;
    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber yawsub_;
  ros::Subscriber optisub_;

  geometry_msgs::Twist output;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");


  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
