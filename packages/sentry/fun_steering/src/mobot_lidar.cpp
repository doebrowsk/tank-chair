// wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <ros/console.h>
#include <math.h>

#include <nav_msgs/Odometry.h>

//wall following is need for both A-Star and the bug algorithm, so we
//want the robot to follow the wall closer than the CORRECTION_THRESHOLD_DISTANCE

const double MIN_SAFE_DISTANCE = 2.0; // set alarm if anything is too close
const double CORRECTION_THRESHOLD_DISTANCE = 2.5;
const double MAX_WALL_FOLLOW_DISTANCE = 4.0;

// these values to be set within the laser callback
float ping_dist_ = 3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;

double last_known_psi_ = 0.0;
double danger_turn_increment_ = 0.5*M_PI/180;
double standard_turn_increment_ = 0.2*M_PI/180;
bool found_wall = false; //needed to determine if we've found a wall yet for the bug algorithm


ros::Publisher desired_state_publisher_;

nav_msgs::Odometry current_des_state_;

// a useful conversion function: from quaternion to yaw
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
	double quat_z = quaternion.z;
	double quat_w = quaternion.w;
	double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
	return phi;
}

//and the other direction:
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
	geometry_msgs::Quaternion quaternion;
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sin(phi / 2.0);
	quaternion.w = cos(phi / 2.0);
	return quaternion;
}

void gazeboPoseCallback(const geometry_msgs::Pose& gazebo_pose) {
	//state_x_ = gazebo_pose.position.x; //copy the state to member variables of this object
	//state_y_ = gazebo_pose.position.y;
	last_known_psi_ = convertPlanarQuat2Phi(gazebo_pose.orientation);
}

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

	bool too_close = false;
	double smallest_ping_dist = HUGE_VAL;
	double smallest_ping = -1;

	for (int i = 0; i <= M_PI/angle_increment_ + 1; i++) {

		ping_index_ = (int) (((-angle_min_ - M_PI/2) + angle_increment_*i)/angle_increment_);

		ping_dist_ = laser_scan.ranges[ping_index_];

		if (ping_dist_ < smallest_ping_dist) {
			smallest_ping_dist = ping_dist_;
			smallest_ping = ping_index_;
		}

		//use cosine so that obstacles close to the side of the robot are less of a concern than those
		//directly in front of the robot
		//added 0.01 so that the min safe distance never goes all the way to 0
		if (ping_dist_ < MIN_SAFE_DISTANCE * cos(angle_increment_ * i - M_PI/2) + 0.01) {
			ROS_WARN("DANGER!!!!!!!!!!!!");

			//start turning away from obstacle
			current_des_state_.pose.pose.orientation = convertPlanarPhi2Quaternion(last_known_psi_ + danger_turn_increment_);
			too_close = true;
			found_wall = true;
			break;
		}
	}

	if (!too_close) {

		if (smallest_ping_dist > MAX_WALL_FOLLOW_DISTANCE) {
			//go straight if we're far from any wall
			ROS_INFO("1");

			//current_des_state_.pose.pose.orientation = convertPlanarPhi2Quaternion(last_known_psi_);
		}
		else if (smallest_ping_dist > CORRECTION_THRESHOLD_DISTANCE) {
			ROS_INFO("2");


			//turn back towards a wall if we're close to a wall by not at close as we'd like.
			//But only if we've made it to the wall by now
			if (found_wall) {
				//current_des_state_.pose.pose.orientation = convertPlanarPhi2Quaternion(last_known_psi_ - standard_turn_increment_);
			}
		}
		else {
			ROS_INFO("3");

			//otherwise, we're close to the wall, but not too close, so travel parallel to the wall

			//look at the pings next to the closest ping to find parallel angle
			int prev_ping = smallest_ping - 1;
			int next_ping = smallest_ping + 1;

			double smallest_ping_theta = last_known_psi_ - (smallest_ping*angle_increment_ - M_PI/2);

			ROS_INFO("smallest_ping_theta, smallest_ping_dist = %f, %f",smallest_ping_theta, smallest_ping_dist);


			double desired_theta;

			if (prev_ping >= 0) {
				double prev_ping_dist = laser_scan.ranges[prev_ping];
				double prev_ping_theta = last_known_psi_ - (prev_ping*angle_increment_ - M_PI/2);

				desired_theta = atan2(prev_ping_dist*sin(prev_ping_theta) - smallest_ping_dist*sin(smallest_ping_theta),
						prev_ping_dist*cos(prev_ping_theta) - smallest_ping_dist*cos(smallest_ping_theta));
				ROS_INFO("desired_theta, prev_ping_dist = %f, %f",desired_theta, prev_ping_dist);
			}

			if (next_ping <= round((angle_max_ - angle_min_)/angle_increment_)) {
				double next_ping_dist = laser_scan.ranges[next_ping];
				double next_ping_theta = last_known_psi_ - (next_ping*angle_increment_ - M_PI/2);

				double tmp_theta = atan2(smallest_ping_dist*sin(smallest_ping_theta) - next_ping_dist*sin(next_ping_theta),
						smallest_ping_dist*cos(smallest_ping_theta) - next_ping_dist*cos(next_ping_theta));
				ROS_INFO("tmp_theta, next_ping_dist = %f, %f",tmp_theta, next_ping_dist);

				if (prev_ping >= 0) {
					//take the average of the theta's
					desired_theta = (desired_theta + tmp_theta)/2;
				}
				else {
					desired_theta = tmp_theta;
				}
			}

			ROS_INFO("desired_theta = %f",desired_theta);

			current_des_state_.pose.pose.orientation = convertPlanarPhi2Quaternion(desired_theta);
		}
	}

	std_msgs::Bool lidar_alarm_msg;
	lidar_alarm_msg.data = laser_alarm_;

	desired_state_publisher_.publish(current_des_state_);

}

int main(int argc, char **argv) {

	current_des_state_.twist.twist.linear.x = 0.0;
	current_des_state_.twist.twist.linear.y = 0.0;
	current_des_state_.twist.twist.linear.z = 0.0;
	current_des_state_.twist.twist.angular.x = 0.0;
	current_des_state_.twist.twist.angular.y = 0.0;
	current_des_state_.twist.twist.angular.z = 0.0;

	current_des_state_.pose.pose.position.x = 0.0;
	current_des_state_.pose.pose.position.y = 0.0;
	current_des_state_.pose.pose.position.z = 0.0;

	current_des_state_.pose.pose.orientation = convertPlanarPhi2Quaternion(0.0);

	ros::init(argc, argv, "lidar_alarm"); //name this node
	ros::NodeHandle nh;

	ros::Subscriber current_state_subscriber_ = nh.subscribe("gazebo_mobot_pose", 1, gazeboPoseCallback);

	//create a Subscriber object and have it subscribe to the lidar topic
	desired_state_publisher_ = nh.advertise<nav_msgs::Odometry>("/desState", 1, true);
	ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, laserCallback);
	ros::spin(); //this is essentially a "while(1)" statement, except it
	// forces refreshing wakeups upon new data arrival
	// main program essentially hangs here, but it must stay alive to keep the callback function alive
	return 0; // should never get here, unless roscore dies
}

