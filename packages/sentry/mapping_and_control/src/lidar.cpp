#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <ros/console.h>
#include <math.h>

#include <nav_msgs/Odometry.h>

const double MIN_SAFE_DISTANCE_FRONT = 1.0; // set alarm if anything is too close

// these values to be set within the laser callback
float ping_dist_ = 3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool lidar_alarm_=false;

ros::Publisher lidar_alarm_publisher_;

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {

	if (ping_index_<0)  {
		//for first message received, set up the desired index of LIDAR range to eval
		angle_min_ = laser_scan.angle_min;
		angle_max_ = laser_scan.angle_max;
		angle_increment_ = laser_scan.angle_increment;
		range_min_ = laser_scan.range_min;
		range_max_ = laser_scan.range_max;
		// what is the index of the ping that is straight ahead?
		// BETTER would be to use transforms, which would reference how the LIDAR is mounted;
		// but this will do for simple illustration
		// ROS_INFO("%f",angle_min_);
		// ROS_INFO("%f",angle_max_);
		// ROS_INFO("%f",angle_increment_);
		// ROS_INFO("%f",range_min_);
		// ROS_INFO("%f",range_max_);
	}

	lidar_alarm_ = false;
	for (int i = 0; i <= M_PI/angle_increment_ + 1; i++) {

		ping_index_ = (int) (((-angle_min_ - M_PI/2) + angle_increment_*i)/angle_increment_);

		ping_dist_ = laser_scan.ranges[ping_index_];

		//use cosine so that obstacles close to the side of the robot are less of a concern than those
		//directly in front of the robot
		//added a bit more so that the min safe distance never goes all the way to 0
		if (ping_dist_ < MIN_SAFE_DISTANCE_FRONT * cos(angle_increment_ * i - M_PI/2) + 0.5) {
			lidar_alarm_ = true;
			break;
		}
	}

	std_msgs::Bool lidar_alarm_msg;
	lidar_alarm_msg.data = lidar_alarm_;
	lidar_alarm_publisher_.publish(lidar_alarm_msg);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "lidar_alarm"); //name this node
	ros::NodeHandle nh;

	//create a Subscriber object and have it subscribe to the lidar topic
    lidar_alarm_publisher_ = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);

	ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, laserCallback);
	ros::spin(); //this is essentially a "while(1)" statement, except it
	// forces refreshing wakeups upon new data arrival
	// main program essentially hangs here, but it must stay alive to keep the callback function alive
	return 0; // should never get here, unless roscore dies
}

