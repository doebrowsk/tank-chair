#ifndef BABS_SLAM_H_
#define BABS_SLAM_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64.h> 
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <vector>


class babs_slam
{
public:
	babs_slam(ros::NodeHandle* nodehandle);

	static void encoder_callback(const nav_msgs::Odometry& odom_value);
	static void lidar_callback(const sensor_msgs::LaserScan& laser_scan);

	//TODO implement callbacks for these with correct message types
	static void imu_callback(const std_msgs::Float64& message_holder);
	static void gps_callback(const std_msgs::Float64& message_holder);

	float pHit(float z, float trueZ);
	float pShort(float z, float trueZ);
	float pMax(float z);
	float pRand(float z);

private:

	geometry_msgs::Pose sampleMotionModel(nav_msgs::Odometry state, double params[]);
	double sample_normal(double bSquared);
	double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
	geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

	static const int NUMPARTICLES = 500;
	ros::NodeHandle nh_;
	std::vector<geometry_msgs::Pose> particles;
	nav_msgs::OccupancyGrid map;

	// Measurement model parameters
	const float Z_HIT = 1;
	const float Z_SHORT = 0;
	const float Z_MAX = 0;
	const float Z_RAND = 0;
	const float STD_HIT = 0.05;
	const float L_SHORT = 0.1;
	const float MAX_LIDAR_RANGE = 8.1;

	void update();
	geometry_msgs::Pose sampleMotionModel(geometry_msgs::Pose p);
	float measurementModelMap(geometry_msgs::Pose p);
	void updateMap();
	void resample(std::vector<float> weights);



	bool compareFloats(float a, float b);
	
	//TODO use a timer or something instead of hard coding
	double dt = 0.1;// time since last running of SLAM,

};

#endif
