#include "ros/ros.h"
#include "ros_cv/LosePointSrv.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "for_lose_num_client");
    if (argc != 2)
    {
        ROS_INFO("usage: for_lose_num_client true/false");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<ros_cv::LosePointSrv>("for_lose_num");
    ros_cv::LosePointSrv srv;

    //if(argv[1] == "true")                         //!!!!!---it's wrong---!!!!
    if(!strcmp("true",argv[1]))                     //1、若参数1>参数2，返回正数；2、若参数1<参数2，返回负数；3、若参数1=参数2，返回0；
    {
        srv.request.getpoint = true;
    }    
    else
    {
        srv.request.getpoint = false;
        ROS_INFO("usage: for_lose_num_client true");
        return 1;
    }

  //srv.request.getpoint = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("lose num = %d", (long int)srv.response.num);
    ROS_INFO("lose position1 x = %f", srv.response.point1.x);
    ROS_INFO("lose position1 y = %f", srv.response.point1.y);
    ROS_INFO("lose position2 x = %f", srv.response.point2.x);
    ROS_INFO("lose position2 y = %f", srv.response.point2.y);
  }
  else
  {
    ROS_ERROR("Failed to call service for_lose_num");
    return 1;
  }

  return 0;
}