#include <ros/ros.h>
#include "std_msgs/String.h"
#include "ardrone_autonomy/Navdata.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "cmath"

#define PUBTOPIC "/vis_to_combine"
#define ARDRONETOPIC "/ardrone/navdata"
#define ROOMBATOPIC "/tobeCOMPLETED"

class YawMessage
{
public:
YawMessage()
{
    pubyaw_ = n_.advertise<std_msgs::Float64>(PUBTOPIC, 1);
    
    CtrlLaw = 0;
    //Topic you want to subscribe
    ardronesub = n_.subscribe(ARDRONETOPIC, 1, &YawMessage::callback, this);
    ardronesub = n_.subscribe(ROOMBATOPIC, 1, &YawMessage::callback2, this);
}

void callback(const ardrone_autonomy::Navdata& msg)
{
//  geometry_msgs::Twist zerotwist;
if (msg.tags_count==0) {return;}

if(CtrlLaw ==0)
{
    msgyaw.data = msg.tags_orientation[0];
    pubyaw_.publish(msgyaw);
} // end of CtrlLaw ==0
else if(CtrlLaw == 1)
{
    msgyaw.data = msg.tags_orientation[0]+R_yawvel;
    // Need to do more processing here
    pubyaw_.publish(msgyaw);
}// end of CtrlLaw ==1
else if(CtrlLaw == 2)
{
    ROS_INFO("This still needs to be coded");
    msgyaw.data = msg.tags_orientation[0]+R_yawvel+0.7*R_yawacc;
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
  double R_yawvel, R_yawacc, CtrlLaw;
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
