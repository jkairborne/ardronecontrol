
#include <iostream>
#include <string>
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <vector>

#include "geometry_msgs/TransformStamped.h"
#include "OptiTools.h"

#define M_PI 3.14159265358979323846

#define OBJECTONE "Ardrone"
#define OBJECTTWO "Roomba"


// This class subscribes to a vrpn broadcast, then re-publishes in the correct xyz frame - this still has the quaternion
class Rebr_rpy
{
public:
  Rebr_rpy(const std::string& msg)
  {
    //Topic to be published
    pub_ = n_.advertise<geometry_msgs::TwistStamped>("/rebr_"+msg+"_rpy", 1);
    //Topic you want to subscribe
    sub_ = n_.subscribe("/vrpn_client_node/"+msg+"/pose", 1, &Rebr_rpy::callback, this);
    old_y = 0.0;
    new_y = 0.0;
    yaw_bias=0.0;
  }

  // This is the main callback function, which accepts a PoseStamped input and then rebroadcasts
  void callback(const geometry_msgs::PoseStamped& input)
  {
      // Create the PoseStamped output message
    geometry_msgs::TwistStamped output;
    // Maintain the same header
    output = Opti_Rect_rpy(input);
    new_y = output.twist.angular.z;

    if(old_y - new_y > 6.0)
    {
        yaw_bias += 2*M_PI;
    }
    else if (old_y - new_y < -6.0)
    {
        yaw_bias -=2*M_PI;
    }

    output.twist.angular.z += yaw_bias;

    old_y = new_y;
    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  double old_y, new_y, yaw_bias;

};//End of class Rebr_rpy

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "rebroadcast_rpy");


  //Create an object of class Rebr_rpy for the Ardrone 
  Rebr_rpy Ardrone(OBJECTONE);
  //Create an object of class Rebr_rpy for the Roomba
  Rebr_rpy Roomba2(OBJECTTWO);

  ros::spin();

  return 0;
}
