#include "pid.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <math.h>

// Here I use global publisher and subscriber, since I want to access the
// publisher in the function MsgCallback:
ros::Publisher quad_twist;
ros::Subscriber quat_subscriber;

// Function for conversion of quaternion to roll pitch and yaw. The angles
// are published here too.
void MsgCallback(const geometry_msgs::PoseStamped msg)
{
    geometry_msgs::Quaternion GMquat;
    GMquat = msg.pose.orientation;

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat, quattemp;
    tf::quaternionMsgToTF(GMquat, quattemp);
//    ROS_INFO("quat.x =%f, quat.y=%f, quat.z=%f, quat.w=%f", quattemp.x(), quattemp.y(), quattemp.z(),quattemp.w());
    quat = tf::Quaternion(quattemp.x(),-quattemp.z(),quattemp.y(),quattemp.w());


    // the tf::Quaternion has a method to acess roll pitch and yaw

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);


    double x,y,z;
    x = cos(yaw)*msg.pose.position.x + sin(yaw)*msg.pose.position.y;
    y = -sin(yaw)*msg.pose.position.x + cos(yaw)*msg.pose.position.y;
    z = msg.pose.position.z;

    geometry_msgs::Twist pid_output;

    PID pidx = PID(0.1,1,-1,1,0,0);
    pid_output.linear.x = pidx.calculate(0,x);
    PID pidy = PID(0.1,1,-1,1,0,0);
    pid_output.linear.y = pidy.calculate(0,y);

    PID pidz = PID(0.1,1,-1,1,0,0);
    pid_output.linear.z = pidz.calculate(0,z);

    ROS_INFO("published x=%.2f, y=%.2f, z=%.2f, p=%.1f, r=%.1f,  yaw=%.1f", pid_output.linear.x,pid_output.linear.y,pid_output.linear.z, pitch*180/3.1415926, roll*180/3.1415926, yaw*180/3.1415926);

    // publish PID output:
    quad_twist.publish(pid_output);


    }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ArdronePID");
    ros::NodeHandle n;
    quad_twist = n.advertise<geometry_msgs::Twist>("angles_output", 1000);
    quat_subscriber = n.subscribe("vrpn_client_node/Ardrone/pose", 1000, MsgCallback);

    // check for incoming quaternions untill ctrl+c is pressed
    ROS_INFO("waiting for quaternion");

    ros::spin();


    return 0;
}
