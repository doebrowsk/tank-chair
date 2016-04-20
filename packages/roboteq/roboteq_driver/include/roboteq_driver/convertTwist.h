#ifndef CONVERT_TWIST
#define CONVERT_TWIST
#include "ros/ros.h"
#include <geometry_msgs/Twist.h> 
class convertTwist{
	public: 
		convertTwist();
		void myCallback(const geometry_msgs::Twist& message_holder);
	protected:
		ros::NodeHandle nh_;
};

#endif