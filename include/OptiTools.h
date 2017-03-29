#ifndef _OptiTools_h
#define _OptiTools_h

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

geometry_msgs::PoseStamped Opti_Rect_quat(const geometry_msgs::PoseStamped& input)
  {
    // Create the PoseStamped output message
    geometry_msgs::PoseStamped output;
    // Maintain the same header
    output.header.seq= input.header.seq;
    // Re-map the x->x, y->z, z->-y (from input to output)...x->x, y->-z, z->y (from output to input)
    output.pose.position.x = input.pose.position.x;
    output.pose.position.y = -input.pose.position.z;
    output.pose.position.z = input.pose.position.y;
    
    // This is necessary because the OptiTrack appears to internally use a left-handed coordinate system.
    // The switching and inversion of the y and z components of the output pose appear to fix this.
    output.pose.orientation.x = input.pose.orientation.x;
    output.pose.orientation.y = -input.pose.orientation.z;
    output.pose.orientation.z = input.pose.orientation.y;
    output.pose.orientation.w = input.pose.orientation.w;

    return output;
}//End of function Opti_Rect

geometry_msgs::TwistStamped Opti_Rect_rpy(const geometry_msgs::PoseStamped& input)
  {
    // Create the PoseStamped output message
      geometry_msgs::TwistStamped output;
    // Maintain the same header
    output.header= input.header;
    // Re-map the x->x, y->z, z->-y (from input to output)...x->x, y->-z, z->y (from output to input)
    output.twist.linear.x = input.pose.position.x;
    output.twist.linear.y = -input.pose.position.z;
    output.twist.linear.z = input.pose.position.y;
    
    // This is necessary because the OptiTrack appears to internally use a left-handed coordinate system.
    // The switching and inversion of the y and z components of the output pose appear to fix this.

    geometry_msgs::PoseStamped temp;
    geometry_msgs::Quaternion GMquat;

    temp.pose.orientation.x = input.pose.orientation.x;
    temp.pose.orientation.y = -input.pose.orientation.z;
    temp.pose.orientation.z = input.pose.orientation.y;
    temp.pose.orientation.w = input.pose.orientation.w;
    GMquat = temp.pose.orientation;
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(GMquat, quat);
            
    // the tf::Quaternion has a method to acess roll pitch and yaw, which we use here
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    output.twist.angular.x = roll;
    output.twist.angular.y = pitch;
    output.twist.angular.z = yaw;

    return output;
}//End of function Opti_Rect
#endif
