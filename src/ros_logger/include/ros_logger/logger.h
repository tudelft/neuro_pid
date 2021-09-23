
#ifndef LOGGER_H_
#define LOGGER_H_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <string>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <iostream>
#include <fstream>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/TorqueThrust.h>
#include <std_msgs/Bool.h>

namespace ros_logger
{

class Server {
public:
    Server(ros::NodeHandle & nh, ros::NodeHandle nh_private);
    virtual ~Server();

private:
    ros::NodeHandle nh_;
    bool simulationStarted;
    void startedCallback(const std_msgs::Bool::ConstPtr& msg);
    // void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void odoCallback(const nav_msgs::Odometry::ConstPtr& msg);
    // void motorCallback(const mav_msgs::Actuators::ConstPtr& msg);
    void thrustCallback(const mav_msgs::TorqueThrust::ConstPtr& msg);
    void waypointCallback(const geometry_msgs::Point::ConstPtr& msg);

    // void logCallback(const sensor_msgs::Imu::ConstPtr& msg, 
	// 							const sensor_msgs::Imu::ConstPtr& groundtruth_msg, 
	// 							const nav_msgs::Odometry::ConstPtr& odo_msg,
	// 							const mav_msgs::Actuators::ConstPtr& motor_msg,
	// 							const mav_msgs::TorqueThrust::ConstPtr& thrust_msg);

    // message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    // message_filters::Subscriber<sensor_msgs::Imu> imu_ground_truth_sub_;
    // message_filters::Subscriber<nav_msgs::Odometry> odometry_sub_;
    // message_filters::Subscriber<mav_msgs::Actuators> motor_speed_sub_;
    // message_filters::Subscriber<mav_msgs::TorqueThrust> thrust_sub_;
    ros::Subscriber started_sub_;
    // ros::Subscriber imu_sub_;
    ros::Subscriber odometry_sub_;
    // ros::Subscriber motor_speed_sub_;
    ros::Subscriber thrust_sub_;
    ros::Subscriber waypoint_sub_;
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu, nav_msgs::Odometry, mav_msgs::Actuators, mav_msgs::TorqueThrust> ImuSyncPolicy_;
    // typedef message_filters::Synchronizer<ImuSyncPolicy_> ImuSync_;
    // boost::shared_ptr<ImuSync_> sync_;

    std::string bag_filename;
    rosbag::Bag DATA_BAG;
};

} // namespace

#endif // LOGGER_H_