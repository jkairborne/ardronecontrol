#include <ros/ros.h>
#include "std_msgs/String.h"
#include "ardrone_autonomy/Navdata.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "cmath"

#define PUBTOPIC "/visual_yaw"
#define ARDRONETOPIC "/ardrone/navdata"
#define ROOMBATOPIC "/roomba_values"

class YawMessage
{
public:
YawMessage()
{
    pubyaw_ = n_.advertise<std_msgs::Float64>(PUBTOPIC, 1);
    
    // Specify which control law to use
    CtrlLaw =2;

    // Define constant gain parameters
    Kp1 = 0.02;
    Kp2 = 0.1;
    Kp3 = 1;

    //Topic you want to subscribe
    ardronesub = n_.subscribe(ARDRONETOPIC, 1, &YawMessage::callback, this);
    roombasub = n_.subscribe(ROOMBATOPIC, 1, &YawMessage::callback2, this);
}

void callback(const ardrone_autonomy::Navdata& msg)
{
//  geometry_msgs::Twist zerotwist;
if (msg.tags_count==0) {return;}

ROS_INFO("in callback for pubyaw");
if(CtrlLaw ==0)
{
    msgyaw.data = Kp1*msg.tags_orientation[0];
    pubyaw_.publish(msgyaw);
    ROS_INFO("In callback for pubyaw, where I should be");
} // end of CtrlLaw ==0
else if(CtrlLaw == 1)
{
    ROS_INFO("In callback for pubyaw for law 2");
    msgyaw.data = Kp1*msg.tags_orientation[0]+Kp2*R_yawvel;
    // Need to do more processing here
    pubyaw_.publish(msgyaw);
}// end of CtrlLaw ==1
else if(CtrlLaw == 2)
{
    ROS_INFO("This still needs to be coded");
    msgyaw.data = Kp1*msg.tags_orientation[0]+Kp2*R_yawvel+Kp3*R_yawacc;
    pubyaw_.publish(msgyaw);
}

} // End of the callback


// This one receives data from the roomba about its velocity and acceleration
void callback2(const geometry_msgs::Vector3& msg)
{
    // msg.x would be the position, but we don't get this information with a Roomba
    R_yawvel = msg.y;
    R_yawacc = msg.z;
}


private:
  ros::NodeHandle n_; 
  ros::Publisher pubyaw_;
  ros::Subscriber ardronesub;
  ros::Subscriber roombasub;
  std_msgs::Float64 msgyaw;
  double R_yawvel, R_yawacc, CtrlLaw, Kp1, Kp2, Kp3;
};//End of class YawMessage

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "Visual_yaw");

  //Create an object of class YawMessage that will take care of everything
  YawMessage YawObject;

  ros::spin();

  return 0;
}
