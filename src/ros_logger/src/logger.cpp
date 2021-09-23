#include "ros_logger/logger.h"

namespace ros_logger {

	Server::Server(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh){
		// setup subscribers and publishers
		simulationStarted = false;
		started_sub_ = nh_.subscribe("simulation_started", 1, &Server::startedCallback, this);
		// imu_sub_ = nh_.subscribe("imu", 1, &Server::imuCallback, this);
		odometry_sub_ = nh_.subscribe("/ground_truth/odometry", 5, &Server::odoCallback, this, ros::TransportHints().tcpNoDelay());
		// motor_speed_sub_ = nh_.subscribe("command/motor_speed", 1, &Server::motorCallback, this);
		thrust_sub_ = nh_.subscribe("/command/torquethrust", 5, &Server::thrustCallback, this, ros::TransportHints().tcpNoDelay());
		waypoint_sub_ = nh_.subscribe("/waypoints", 1, &Server::waypointCallback, this);

        // sync_.reset(new ImuSync_(ImuSyncPolicy_(100), imu_sub_, imu_ground_truth_sub_, odometry_sub_, motor_speed_sub_, thrust_sub_));
        // sync_->registerCallback(boost::bind(&ros_logger::Server::logCallback, this, _1, _2, _3, _4, _5));

		nh_private.getParam("filename", bag_filename);
		// data record in .bag
		DATA_BAG.open(bag_filename + ".bag", rosbag::bagmode::Write);
	}

	Server::~Server(){
		/** Closing .bag files */
		DATA_BAG.close();
	}

	void Server::startedCallback(const std_msgs::Bool::ConstPtr& msg) {
		simulationStarted = true;
	}

	// void Server::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	// 	if (simulationStarted) {
	// 		DATA_BAG.write("imu", ros::Time::now(), msg);
	// 	}
	// }

	void Server::odoCallback(const nav_msgs::Odometry::ConstPtr& msg) {
		if (simulationStarted) {
			DATA_BAG.write("/optitrack", ros::Time::now(), msg);
		}
	}

	// void Server::motorCallback(const mav_msgs::Actuators::ConstPtr& msg) {
	// 	if (simulationStarted) {
	// 		DATA_BAG.write("command/motor_speed", ros::Time::now(), msg);
	// 	}
	// }

	void Server::thrustCallback(const mav_msgs::TorqueThrust::ConstPtr& msg) {
		if (simulationStarted) {
			DATA_BAG.write("/torquethrust", ros::Time::now(), msg);
		}
	}

	void Server::waypointCallback(const geometry_msgs::Point::ConstPtr& msg) {
		if (simulationStarted) {
			DATA_BAG.write("/waypoints", ros::Time::now(), msg);
		}
	}


    // void Server::logCallback(const sensor_msgs::Imu::ConstPtr& msg, 
	// 							const sensor_msgs::Imu::ConstPtr& groundtruth_msg, 
	// 							const nav_msgs::Odometry::ConstPtr& odo_msg,
	// 							const mav_msgs::Actuators::ConstPtr& motor_msg,
	// 							const mav_msgs::TorqueThrust::ConstPtr& thrust_msg){
    //     DATA_BAG.write("imu", ros::Time::now(), msg);
    //     DATA_BAG.write("ground_truth/imu", ros::Time::now(), groundtruth_msg);
	// 	DATA_BAG.write("ground_truth/odometry", ros::Time::now(), odo_msg);
	// 	DATA_BAG.write("command/motor_speed", ros::Time::now(), motor_msg);
	// 	DATA_BAG.write("command/torquethrust", ros::Time::now(), thrust_msg);
    // }
} // namespace