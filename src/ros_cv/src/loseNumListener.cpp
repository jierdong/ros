#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_cv/LosePointMsg.h"


//callback function
void chatterCallback(const ros_cv::LosePointMsg::ConstPtr& msg)
{
    ROS_INFO("lose num =  [%d]", msg->num);
    ROS_INFO("lose point1 x =  [%f]", msg->point1.x);
    ROS_INFO("lose point1 y =  [%f]", msg->point1.y);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "losenumlistener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter1", 1000, chatterCallback);      //chatter话题名 1000缓存条数
  ros::spin();          //进入自循环，可以尽可能快的调用消息回调函数
  return 0;
}
