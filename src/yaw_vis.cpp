#include <ros/ros.h>
#include "std_msgs/String.h"
#include "ardrone_autonomy/Navdata.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "cmath"

#include "pid.h"
#include <dynamic_reconfigure/server.h>
#include <ardronecontrol/PIDsetConfig.h>

#define PUBTOPIC "/visual2aw"
#define ARDRONETOPIC "/ardrone/navdata"
#define ROOMBATOPIC "/roomba_values"

// Function prototypes which are defined after main. First is to clip output values to the -1<1 range. 
// Second is the callback for the parameter server function.
float clip(float n, float lower, float upper);
void callback(ardronecontrol::PIDsetConfig &config, uint32_t level) ;
ros::Publisher new_gains_yaw;

//PIDs for the rate and acceleration in yaw. Sample time is approximately 1/30, since camera detection updates @ 30Hz.
    PID pid1 = PID((1/30),1,-1,1,0,0);
    PID pid2 = PID((1/30),1,-1,1,0,0);
    PID pid3 = PID((1/30),1,-1,1,0,0);

class YawMessage
{
public:
YawMessage()
{
    pubyaw_ = n_.advertise<std_msgs::Float64>(PUBTOPIC, 1);
    
    // Specify which control law to use
    CtrlLaw =1;

    //Topic you want to subscribe
    ardronesub = n_.subscribe(ARDRONETOPIC, 1, &YawMessage::callback, this);
    roombasub = n_.subscribe(ROOMBATOPIC, 1, &YawMessage::callback2, this);
}

void callback(const ardrone_autonomy::Navdata& msg)
{
//  geometry_msgs::Twist zerotwist;
if (msg.tags_count==0) 
{
    msgyaw.data = 0;
    pubyaw_.publish(msgyaw);
    return;
}

yaw = msg.tags_orientation[0];
if(yaw >180){yaw-=360;}

if(CtrlLaw ==0)
{
    msgyaw.data = pid1.calculate(0,yaw,(1/30));
} // end of CtrlLaw ==0
else if(CtrlLaw == 1)
{
    // Need to do more processing here
    msgyaw.data = pid1.calculate(0,yaw,(1/30)) + pid2.calculate(0,yaw,(1/30));
}// end of CtrlLaw ==1
else if(CtrlLaw == 2)
{
    msgyaw.data = pid1.calculate(0,yaw,(1/30)) + pid2.calculate(0,yaw,(1/30)) + pid3.calculate(0,yaw,(1/30));
}
    msgyaw.data = clip(msgyaw.data,-1,+1);
    pubyaw_.publish(msgyaw);

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
  double R_yawvel, R_yawacc, CtrlLaw, Kp1, Kp2, Kp3, yaw;
};//End of class YawMessage

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "Visual2aw");
    ros::NodeHandle n;

    new_gains_yaw = n.advertise<geometry_msgs::TwistStamped>("gain_change_yaw", 5);
    //Create an object of class YawMessage that will take care of everything
    YawMessage YawObject;

    dynamic_reconfigure::Server<ardronecontrol::PIDsetConfig> server;
    dynamic_reconfigure::Server<ardronecontrol::PIDsetConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

  ros::spin();

  return 0;
}




void callback(ardronecontrol::PIDsetConfig &config, uint32_t level) {
//  ROS_INFO("Reconfigure Request: %f %f", 
//             config.Kp1,config.set1);

  double Kp1, Kd1, Ki1, Kp2, Kd2, Ki2;
// Save the new configuration to doubles
  Kp1 = config.Kp_x;
  Kd1 = config.Kd_x;
  Ki1 = config.Ki_x;
  // Call the mod_params function of the Pimpl class - this then calls the set_gains function in the PID class which actually changes the gains used for calculations
  pid1.mod_params(Kp1, Kd1,Ki1);

// Save the new configuration to doubles
  Kp2 = config.Kp_y;
  Kd2 = config.Kd_y;
  Ki2 = config.Ki_y;
  // Call the mod_params function of the Pimpl class - this then calls the set_gains function in the PID class which actually changes the gains used for calculations
  pid2.mod_params(Kp2, Kd2,Ki2);

  // Create and publish the new gains to a TwistStamped method:
  geometry_msgs::TwistStamped newgains;
  newgains.header.stamp = ros::Time::now();
  newgains.twist.linear.x = Kp1;
  newgains.twist.linear.y = Kd1;
  newgains.twist.linear.z = Ki1;
  newgains.twist.angular.x = Kp2;
  newgains.twist.angular.y = Kd2;
  newgains.twist.angular.z = Ki2;

  new_gains_yaw.publish(newgains);
} // End callback

// Define a clip function that will clip number n to remain within lower and upper
// In our case, used to keep the output command 
float clip(float n, float lower, float upper) {
      return std::max(lower, std::min(n, upper));
} // End clib
