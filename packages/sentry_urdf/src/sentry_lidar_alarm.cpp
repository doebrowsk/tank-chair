// wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <ros/console.h>
#include <math.h>


const double MIN_SAFE_DISTANCE = 1.5; // set alarm if anything is within 1.5m of the front of robot

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;
double PI = 3.1415;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

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


		ROS_INFO("%f",angle_min_);
		ROS_INFO("%f",angle_max_);
		ROS_INFO("%f",angle_increment_);
		ROS_INFO("%f",range_min_);
		ROS_INFO("%f",range_max_);

		ping_index_ = (int) (0.0 -angle_min_)/angle_increment_;
		ROS_INFO("LIDAR setup: ping_index = %d",ping_index_);

	}

	for (int i = 0; i < PI/angle_increment_; i++) {

		ping_index_ = (int) (((-angle_min_ - PI/2) + angle_increment_*i)/angle_increment_);

		ping_dist_in_front_ = laser_scan.ranges[ping_index_];

		//use cosine so that obstacles close to the side of the robot are less of a concern than those
		//directly in front of the robot
		//added 0.01 so that the min safe distance never goes all the way to 0
		if (ping_dist_in_front_ < MIN_SAFE_DISTANCE * cos(angle_increment_ * i - PI/2) + 0.01) {
			ROS_WARN("DANGER!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
			laser_alarm_=true;
			break;
		}
		else {
			laser_alarm_=false;
		}
	}

	std_msgs::Bool lidar_alarm_msg;
	lidar_alarm_msg.data = laser_alarm_;
	lidar_alarm_publisher_.publish(lidar_alarm_msg);

	std_msgs::Float32 lidar_dist_msg;
	lidar_dist_msg.data = ping_dist_in_front_;
	lidar_dist_publisher_.publish(lidar_dist_msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "lidar_alarm"); //name this node
	ros::NodeHandle nh;
	//create a Subscriber object and have it subscribe to the lidar topic
	ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
	lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
	ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);
	lidar_dist_publisher_ = pub2;
	ros::Subscriber lidar_subscriber = nh.subscribe("scan", 1, laserCallback);
	ros::spin(); //this is essentially a "while(1)" statement, except it
	// forces refreshing wakeups upon new data arrival
	// main program essentially hangs here, but it must stay alive to keep the callback function alive
	return 0; // should never get here, unless roscore dies
}

