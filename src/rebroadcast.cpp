
#include <iostream>
#include <string>
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <vector>

#include "geometry_msgs/TransformStamped.h"

// This class subscribes to a vrpn broadcast, then re-publishes in the correct xyz frame - this still has the quaternion
class SubscribeAndPublish
{
    std::string msg2;
public:
  SubscribeAndPublish(const std::string& msg)
  {
    //Topic to be published
    pub_ = n_.advertise<geometry_msgs::PoseStamped>(("/rebroadcast/"+msg), 1);
    // Need this because or else we will not have access to msg from outside the constructor
    msg2 = msg;
    //Topic you want to subscribe
    sub_ = n_.subscribe("/vrpn_client_node/"+msg, 1, &SubscribeAndPublish::callback, this);
  }

  // This is the main callback function, which accepts a PoseStamped input and then rebroadcasts
  void callback(const geometry_msgs::PoseStamped& input)
  {
      // Create the PoseStamped output message
    geometry_msgs::PoseStamped output;
    // Maintain the same header
    output.header= input.header;
    // Re-map the x->x, y->z, z->-y (from input to output)...x->x, y->-z, z->y (from output to input)
    output.pose.position.x = input.pose.position.x;
    output.pose.position.y = -input.pose.position.z;
    output.pose.position.z = input.pose.position.y;
    output.pose.orientation = input.pose.orientation;


    ROS_INFO("This is another test");
    pub_.publish(output);
    //For testing purposes:
    ROS_INFO("we are here %s",msg2.c_str());
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");


  //Create an object of class SubscribeAndPublish for the Ardrone 
  SubscribeAndPublish SAPObject("/Ardrone/pose");

  ros::spin();

  return 0;
}
