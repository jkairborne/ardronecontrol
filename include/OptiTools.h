#ifndef _OptiTools_h
#define _OptiTools_h

#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"

geometry_msgs::PoseStamped Opti_Rect(const geometry_msgs::PoseStamped& input)
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


geometry_msgs::TransformStamped Opti_Rect(const geometry_msgs::TransformStamped& input)
{
    // Create the TransformStamped output message
    geometry_msgs::TransformStamped output;
     // Maintain the same header
     output.header.seq= input.header.seq;
     // Re-map the x->x, y->z, z->-y (from input to output)...x->x, y->-z, z->y (from output to input)
    output.transform.translation.x = input.transform.translation.x;
    output.transform.translation.y = input.transform.translation.y;
    output.transform.translation.z = input.transform.translation.z;
     
     // This is necessary because the OptiTrack appears to internally use a left-handed coordinate system.
     // The switching and inversion of the y and z components of the output pose appear to fix this.
    output.transform.rotation.x = input.transform.rotation.x;
    output.transform.rotation.y = input.transform.rotation.y;
    output.transform.rotation.z = input.transform.rotation.z;
    output.transform.rotation.w = input.transform.rotation.w;
 
     return output;
 }//End of function Opti_Rect



#endif
