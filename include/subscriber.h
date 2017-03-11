#ifndef _subscriber_h
#define _subscriber_h

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

class Subscriber
{
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  geometry_msgs::PoseStamped Pose_Pr;
  int i;
public:
  Subscriber() 
  {
      ROS_ERROR("created an empty Subscriber");
  }

  Subscriber(const std::string& msg)
  {
    //Topic you want to subscribe
    sub_ = n_.subscribe("/vrpn_client_node/"+msg, 1, &Subscriber::callback, this);
    geometry_msgs::PoseStamped Pose_Pr;
  }

  // This is the main callback function, which accepts a PoseStamped input and then rebroadcasts
  void callback(const geometry_msgs::PoseStamped& input)
  {
    // Maintain the same header
    Pose_Pr.header.seq= input.header.seq;
    // Re-map the x->x, y->z, z->-y (from input to output)...x->x, y->-z, z->y (from output to input)
    Pose_Pr.pose.position.x = input.pose.position.x;
    Pose_Pr.pose.position.y = -input.pose.position.z;
    Pose_Pr.pose.position.z = input.pose.position.y;

    // Make sure the orientation remains unchanged for now
    Pose_Pr.pose.orientation = input.pose.orientation;
    ROS_INFO("We are in the callback");
  }

  geometry_msgs::PoseStamped GetPose()
  {
      return Pose_Pr;
  } // End of GetPose()
};//End of class Subscribee

#endif
