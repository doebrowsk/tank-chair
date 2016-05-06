#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <math.h>

float ping_dist_in_front =3.0;
int ping_index = -1;
double angle_min =0.0;
double angle_max =0.0;
double angle_increment =0.0;
double range_min = 0.0;
double range_max = 0.0;

int motionMode = -1;
ros::Publisher point_publisher;
int stepNum; //keep track of how many points we've sent, needed to determine if we should orient to the goal or not

const int E_STOPPED = 0; //define some mode keywords
const int DONE_W_SUBGOAL = 1;
const int PURSUING_SUBGOAL = 2;
const int HALTING = 3;
const int OFF = 4;

bool moved;
const double barrier_tolerance = 2.0;
geometry_msgs::Point goalPoint;
geometry_msgs::Point currentPoint;

void motionModeCallback(const std_msgs::Int32& motionModeMsg) {
	motionMode = motionModeMsg.data;
}

void currentPoseCallback(const geometry_msgs::PoseStamped& msg) {
	currentPoint.x = msg.pose.position.x;
	currentPoint.y = msg.pose.position.y;
}

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {

	if (ping_index<0)  {
		//for first message received, set up the desired index of LIDAR range to eval
		angle_min = laser_scan.angle_min;
		angle_max = laser_scan.angle_max;
		angle_increment = laser_scan.angle_increment;
		range_min = laser_scan.range_min;
		range_max = laser_scan.range_max;
		// what is the index of the ping that is straight ahead?
		// BETTER would be to use transforms, which would reference how the LIDAR is mounted;
		// but this will do for simple illustration
		ping_index = (int) ((0.0 -angle_min)/angle_increment);

		ROS_INFO("angle_min_: %f",angle_min);
		ROS_INFO("angle_max_: %f",angle_max);
		ROS_INFO("angle_increment_: %f",angle_increment);
		ROS_INFO("range_min_: %f",range_min);
		ROS_INFO("range_max_: %f",range_max);
		ROS_INFO("ping_index_: %d",ping_index);

	}

	switch (motionMode) {
	case OFF: {
		break;
	}
	case E_STOPPED:
	{
		break;
	}
	case HALTING:
	{
		break;
	}
	case PURSUING_SUBGOAL:
	{
		moved = true;
		break;
	}
	case DONE_W_SUBGOAL:
	{
		if (moved) {
			geometry_msgs::Point next_point;
			geometry_msgs::PointStamped point_to_send;
			double xDiff = goalPoint.x - currentPoint.x;
			double yDiff = goalPoint.y - currentPoint.y;
			ROS_INFO("xDiff yDiff %f %f", xDiff,yDiff);

			double distToGoal = sqrt(pow(xDiff,2) + pow(yDiff,2));
			ROS_INFO("goal dist: %f", distToGoal);

			//stop if we're within a meter of the goal
			if(abs(xDiff) < 1 && abs(yDiff) < 1) {
				ROS_INFO("GOAL ACHIEVED!!!");
			}
			else {
				if (stepNum % 2 == 0) {

					//first face the goal
					ROS_INFO("Orienting");

					double moveDist = 1.0;

					double theta = atan2(yDiff,xDiff);

					double deltaY = moveDist*sin(theta);
					double deltaX = moveDist*cos(theta);

					next_point.x = currentPoint.x + deltaX;
					next_point.y = currentPoint.y + deltaY;
				}
				else {
					double last_ping_index = 90;
					double last_ping_dist = laser_scan.ranges[last_ping_index];

					//go to the goal if we can see it
					if (distToGoal < last_ping_dist) {
						ROS_INFO("Tangent bug headed straight for goal");
						next_point.x = goalPoint.x;
						next_point.y = goalPoint.y;
					}
					else {
						ROS_INFO("Tangent bug generating new point");
						bool foundBreakLeft = false;
						bool foundBreakRight = false;
						double num_pings = 180;
						geometry_msgs::Point left_point;
						geometry_msgs::Point right_point;

						//scan to the left
						for (int i = 90; i <= num_pings; i++) {
							double ping_dist = laser_scan.ranges[i];

							//look for break in barrier
							double diff = abs(ping_dist - last_ping_dist);

							if (diff > barrier_tolerance) {
								ROS_INFO("Found a left break at index: %d, (ping_dist,last_ping_dist) = (%f,%f)",i,ping_dist,last_ping_dist);
								foundBreakLeft = true;
								break;
							}
							last_ping_dist = ping_dist;
							last_ping_index = i;
						}

						if (foundBreakLeft) {
							double moveDist = last_ping_dist + 2.0;
							double theta = (last_ping_index - 90 + 1)*angle_increment;
							ROS_INFO("theta left is %f",theta);

							double deltaX = -moveDist*sin(theta);
							double deltaY = moveDist*cos(theta);

							ROS_INFO("left deltaX,deltaY = %f,%f",deltaX,deltaY);

							left_point.x = currentPoint.x + deltaX;
							left_point.y = currentPoint.y + deltaY;
						}

						last_ping_index = 90;
						last_ping_dist = laser_scan.ranges[last_ping_index];
						//scan to the right
						for (int i = 90; i >= 0; i--) {
							double ping_dist = laser_scan.ranges[i];

							//look for break in barrier
							double diff = abs(ping_dist - last_ping_dist);

							if (diff > barrier_tolerance) {
								ROS_INFO("Found a right break at index: %d, (ping_dist,last_ping_dist) = (%f,%f)",i,ping_dist,last_ping_dist);
								foundBreakRight = true;
								break;
							}
							last_ping_dist = ping_dist;
							last_ping_index = i;
						}

						if (foundBreakRight) {
							double moveDist = last_ping_dist + 2.0;
							double theta = (90 - last_ping_index + 1)*angle_increment;
							ROS_INFO("theta right is %f",theta);

							double deltaX = moveDist*sin(theta);
							double deltaY = moveDist*cos(theta);

							ROS_INFO("right deltaX,deltaY = %f,%f",deltaX,deltaY);

							right_point.x = currentPoint.x + deltaX;
							right_point.y = currentPoint.y + deltaY;
						}

						if (foundBreakLeft && foundBreakRight) {
							//go to whichever is closer if we have two options
							xDiff = left_point.x - currentPoint.x;
							yDiff = left_point.y - currentPoint.y;
							ROS_INFO("left xDiff yDiff %f %f", xDiff,yDiff);
							double distToLeft = sqrt(pow(xDiff,2) + pow(yDiff,2));

							xDiff = right_point.x - currentPoint.x;
							yDiff = right_point.y - currentPoint.y;
							ROS_INFO("right xDiff yDiff %f %f", xDiff,yDiff);
							double distToRight = sqrt(pow(xDiff,2) + pow(yDiff,2));

							if (distToLeft < distToRight) {
								next_point = left_point;
							}
							else {
								next_point = right_point;
							}
						}
						else if (foundBreakRight) {
							next_point = right_point;
						}
						else if (foundBreakLeft) {
							next_point = left_point;
						}
						else {
							ROS_WARN("unexpected state1!!!!");
						}
					}
				}

				point_to_send.point = next_point;
				ROS_INFO("sending point (%f,%f)",next_point.x,next_point.y);
				point_publisher.publish(point_to_send);
				stepNum++;
				moved = false;
			}
		}
		break;
	}
	default: //should not happen
	{
		ROS_WARN("Tangent bug: Motion mode: (%d) not recognized!", motionMode);
		break;
	}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "tangent_bug"); //name this node
	ros::NodeHandle nh;
	moved = true;;

	goalPoint.x = 20;
	goalPoint.y = 5;

	point_publisher = nh.advertise<geometry_msgs::PointStamped>("bug_point", 1);

	ros::Subscriber lidar_subscriber = nh.subscribe("gps_bug/cspace_scan", 1, laserCallback);
	ros::Subscriber motion_mode_subscriber = nh.subscribe("motion_mode", 1, motionModeCallback);
	ros::Subscriber current_pose_subscriber = nh.subscribe("currentPose", 1, currentPoseCallback);

	ros::spin();
	return 0;
}
