#include "ros_logger/logger.h"

int main(int argc, char* argv[]){
  ros::init(argc, argv, "ros_logger");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros_logger::Server server(nh, nh_private);

  ros::spin();

  return 0;
}