#ifndef _subscriber_h
#define _subscriber_h


class Subscriber
{
public:
  Subscriber(const std::string& msg)
  {
    //Topic you want to subscribe
    sub_ = n_.subscribe("/vrpn_client_node/"+msg, 1, &SubscribeAndPublish::callback, this);
    geometry_msgs::PoseStamped NewPose;
  }

  // This is the main callback function, which accepts a PoseStamped input and then rebroadcasts
  void callback(const geometry_msgs::PoseStamped& input)
  {
    // Maintain the same header
    NewPose.header.seq= input.header.seq;
    // Re-map the x->x, y->z, z->-y (from input to output)...x->x, y->-z, z->y (from output to input)
    NewPose.pose.position.x = input.pose.position.x;
    NewPose.pose.position.y = -input.pose.position.z;
    NewPose.pose.position.z = input.pose.position.y;

    // Make sure the orientation remains unchanged for now
    NewPose.pose.orientation = input.pose.orientation;
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class Subscribee

#endif
