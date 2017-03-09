#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "hello");

//cv::Mat img = cv::imread("~/Pictures/img1.jpg", 1);
cv::Mat img = cv::Mat::zeros(512,512, 0);
cv::imshow("fsda", img);
cv::waitKey(0);

  ROS_INFO("Boo");
}
