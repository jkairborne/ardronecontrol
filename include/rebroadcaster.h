#ifndef _rebroadcaster_h
#define _rebroadcaster_h


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
    output.header.seq= input.header.seq;
    // Re-map the x->x, y->z, z->-y (from input to output)...x->x, y->-z, z->y (from output to input)
    output.pose.position.x = input.pose.position.x;
    output.pose.position.y = -input.pose.position.z;
    output.pose.position.z = input.pose.position.y;

    // Make sure the orientation remains unchanged for now
    output.pose.orientation = input.pose.orientation;

    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

#endif
