
#include <pluginlib/class_list_macros.h>
#include "ros_logger/logger_nodelet.h"

namespace ros_logger{

    void LoggerNodelet::onInit(){
      server = new ros_logger::Server(getNodeHandle(), getPrivateNodeHandle());
      NODELET_INFO_STREAM("Initialized " <<  getName() << " nodelet.");
    }

    #ifndef PLUGINLIB_EXPORT_CLASS
        PLUGINLIB_DECLARE_CLASS(ros_logger, LoggerNodelet, ros_logger::LoggerNodelet, nodelet::Nodelet);
    #else
        PLUGINLIB_EXPORT_CLASS(ros_logger::LoggerNodelet, nodelet::Nodelet);
    #endif

}