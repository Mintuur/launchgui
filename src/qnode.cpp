/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include "../include/launchgui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/



namespace launchgui {

/*****************************************************************************
** Implementation
*****************************************************************************/

int State[5];
int Ready;

extern int ros_topic_data;
extern bool ros_status_flag;

std_msgs::UInt16 msg;

std_msgs::UInt16 Autodriving_state;
std_msgs::UInt16 Door_state;
std_msgs::UInt16 Obstacle_state;
std_msgs::UInt16 Parking_state;
std_msgs::UInt16 Stair_state;

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"launchgui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
        mission_publisher = n.advertise<std_msgs::UInt16>("mission", 1);

        get_Ready_subscriber = n.subscribe("getmission_ready", 1000, &QNode::getready_Callback, this);

        A_state_subscriber = n.subscribe("autodriving_state", 1000,  &QNode::A_state_Callback, this); //mission state
        D_state_subscriber = n.subscribe("door_state", 1000,  &QNode::D_state_Callback, this);
        O_state_subscriber = n.subscribe("obstacle_state", 1000,  &QNode::O_state_Callback, this);
        S_state_subscriber = n.subscribe("stair_state", 1000,  &QNode::S_state_Callback, this);
        P_state_subscriber = n.subscribe("parking_state", 1000,  &QNode::P_state_Callback, this);
        MD_state_subscriber = n.subscribe("md_driver_status", 1000, &QNode::MD_state_Callback, this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"launchgui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
        mission_publisher = n.advertise<std_msgs::UInt16>("mission", 1);

        get_Ready_subscriber = n.subscribe("getmission_ready", 1000, &QNode::getready_Callback, this);

        A_state_subscriber = n.subscribe("autodriving_state", 1000,  &QNode::A_state_Callback, this); //mission state
        D_state_subscriber = n.subscribe("door_state", 1000,  &QNode::D_state_Callback, this);
        O_state_subscriber = n.subscribe("obstacle_state", 1000,  &QNode::O_state_Callback, this);
        S_state_subscriber = n.subscribe("stair_state", 1000,  &QNode::S_state_Callback, this);
        P_state_subscriber = n.subscribe("parking_state", 1000,  &QNode::P_state_Callback, this);
        MD_state_subscriber = n.subscribe("md_driver_status", 1000, &QNode::MD_state_Callback, this);
	start();
	return true;
}

void QNode::run() {
        ros::Rate loop_rate(10);
        ros::NodeHandle n;

        get_Ready_subscriber = n.subscribe("getmission_ready", 1000, &QNode::getready_Callback, this);

        A_state_subscriber = n.subscribe("autodriving_state", 1000,  &QNode::A_state_Callback, this); //mission state
        D_state_subscriber = n.subscribe("door_state", 1000,  &QNode::D_state_Callback, this);
        O_state_subscriber = n.subscribe("obstacle_state", 1000,  &QNode::O_state_Callback, this);
        S_state_subscriber = n.subscribe("stair_state", 1000,  &QNode::S_state_Callback, this);
        P_state_subscriber = n.subscribe("parking_state", 1000,  &QNode::P_state_Callback, this);
        MD_state_subscriber = n.subscribe("md_driver_status", 1000, &QNode::MD_state_Callback, this);


	int count = 0;
        while ( ros::ok() ) {
            if(ros_status_flag == true) {
                msg.data = ros_topic_data;
                mission_publisher.publish(msg);
                ros_status_flag = false;
            }
            blackout(0);

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

        State[0] = 0;

        ros::spin();
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::A_state_Callback(const std_msgs::UInt16& state_msg){
    State[0] = state_msg.data;
        Q_EMIT statusUpdated();
}

void QNode::D_state_Callback(const std_msgs::UInt16& state_msg){
    State[1] = state_msg.data;
        Q_EMIT statusUpdated();
}

void QNode::O_state_Callback(const std_msgs::UInt16& state_msg){
    State[2] = state_msg.data;
        Q_EMIT statusUpdated();
}

void QNode::P_state_Callback(const std_msgs::UInt16& state_msg){
    State[3] = state_msg.data;
        Q_EMIT statusUpdated();
}

void QNode::S_state_Callback(const std_msgs::UInt16& state_msg){
    State[4] = state_msg.data;
        Q_EMIT statusUpdated();
}

void QNode::MD_state_Callback(const std_msgs::UInt16& state_msg){
    State[5] = state_msg.data;
        Q_EMIT statusUpdated();
}

void QNode::blackout(int a){

    for(int i=0; i<6; i++) {
        State[i] = a;
    }
    Ready = a;

    Q_EMIT statusUpdated();
}

void QNode::getready_Callback(const std_msgs::UInt16& ready){
    Ready = ready.data;
        Q_EMIT statusUpdated();
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace launchgui


//md_driver_status int number 1
