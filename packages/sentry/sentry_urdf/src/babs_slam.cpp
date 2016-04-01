#include "babs_slam.h"


babs_slam::babs_slam(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor


}

// Table 5.3 from the book
// Samples a pose from the motion model given previous pose and control
// params assumed to be of length 6
geometry_msgs::Pose babs_slam::sampleMotionModel(nav_msgs::Odometry state, double params[]) {
	// Add noise to the initial pose

	geometry_msgs::Pose pose = state.pose.pose;
	geometry_msgs::Twist twist = state.twist.twist;

	double fwd_vel = twist.linear.x;
	double twist_vel = twist.angular.z;
	double theta = convertPlanarQuat2Phi(pose.orientation);

	double v = fwd_vel + sample_normal(params[0]*pow(fwd_vel,2) + params[1]*pow(twist_vel,2));
	double w = twist_vel + sample_normal(params[2]*pow(fwd_vel,2) + params[3]*pow(twist_vel,2));

	// Calculate the final pose
	double gamma = sample_normal(params[4]*pow(fwd_vel,2) + params[5]*pow(twist_vel,2));
	pose.position.x = pose.position.x - (v/w)*sin(theta) + (v/w)*sin(theta + w*dt);
	pose.position.y = pose.position.y + (v/w)*cos(theta) + (v/w)*cos(theta + w*dt);
	double newTheta = theta + w*dt + gamma*dt;

	pose.orientation = convertPlanarPhi2Quaternion(newTheta);

	return pose;
}

// Table 5.4 from the book
double babs_slam::sample_normal(double bSquared) {
	double b = sqrt(bSquared);
	int result = 0;
	for (int i = 1; i <= 12; i++) {
		result += rand()*2*b - b;//add a rand number between -b and b
	}
	return 0.5*result;
}

// a useful conversion function: from quaternion to yaw
double babs_slam::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
	double quat_z = quaternion.z;
	double quat_w = quaternion.w;
	double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
	return phi;
}

//and the other direction:
geometry_msgs::Quaternion babs_slam::convertPlanarPhi2Quaternion(double phi) {
	geometry_msgs::Quaternion quaternion;
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sin(phi / 2.0);
	quaternion.w = cos(phi / 2.0);
	return quaternion;
}

void babs_slam::update(){
	std::vector<geometry_msgs::Pose> newParticles;
	std::vector<float> particleWeights;
	//result will update particles
	for (int i = 0; i < particles.size();i++){
		geometry_msgs::Pose p = particles[i];
		//get new particles
		geometry_msgs::Pose newpose = sampleMotionModel(p);
		//weigh particles
		particleWeights.push_back(measurementModelMap(newpose));
		updateMap();
	}
	particles=newParticles;
	resample(particleWeights);
}

geometry_msgs::Pose babs_slam::sampleMotionModel(geometry_msgs::Pose p){

	return p;
}

float babs_slam::measurementModelMap(geometry_msgs::Pose p){
	return 0.0;
}

void babs_slam::updateMap(){

}

//// table 9.2 from the book
//// Computes the change in each map cell
////
//// x and y reference the cell of interest in the occupancy grid.
//// Measurement is the entire lidar scan
//// Pose is the Pose of the robot
//float inverseSensorModel(int x, int y, Measurement mt, Pose pose) {
//	// Get the center of the cell
//	float x = cellX + 0.5;
//	float y = cellY + 0.5;
//	// Get distance and angle of (cell-pose) vector
//	float r = sqrt((x-pose.x)^2+(y-pose.y)^2);
//	float phi = atan2(x-pose.x, y-pose.y) - pose.theta
//	// Find the closest ray
//	minAngle = 99999
//	k = 0
//	for (i=0; i<len(mt.values); i++) {
//		float angle = getAngle(pose, i);
//		if (angle < minAngle) {
//			minAngle = angle;
//			k = i;
//		}
//	}
//	z = mt.values[k]
//	// Change cell occupancy depending on the measurement and cell coordinates
//	if (r > min(zMax, z+ismAlpha/2) || abs(phi-minAngle)>ismBeta/2)
//		return l0;
//	if (z<zMax && abs(r-z)<ismAlpha/2)
//		return lOcc;
//	return lFree;
//}


void babs_slam::resample(std::vector<float> weights){
	float total_weight = 0;
	for (int i = 0; i < NUMPARTICLES; i ++){
		//add up the size of all the weights
		total_weight+= weights[i];
	}
	std::vector<float> samples;
	for (int i = 0; i < NUMPARTICLES; i++){
		//take a bunch of random numbers in the range of the weights
		samples.push_back(rand()/RAND_MAX*total_weight);
	}
	std::sort(samples.begin(),samples.end());
	std::vector<geometry_msgs::Pose> newParticles;
	total_weight =  weights[0];
	int weight_counter = 0;
	for (int i = 0; i < NUMPARTICLES; i++){
		if (total_weight>samples[i]){
			newParticles.push_back(particles[weight_counter]);
		}
		else{
			weight_counter ++;
			total_weight += weights[weight_counter];
		}
	}
	particles = newParticles;
}

// Sensor model probabilities

// eq 6.4-6.6 form the book
float babs_slam::pHit(float z, float trueZ) {
	if (z<0 || z>MAX_LIDAR_RANGE)
		return 0;
	float gaussianProb = (1/sqrt(2*M_PI*pow(STD_HIT,2)))*exp(-0.5*pow((z-trueZ),2)/pow(STD_HIT,2));
	float normalizer = 1; // Supposed to be an integral, but it only matters when measured value is close to min or max
	return gaussianProb*normalizer;
}

// eq 6.7-6.9 from the book
float babs_slam::pShort(float z, float trueZ) {
	if (z<0 || z>trueZ || z>MAX_LIDAR_RANGE)
		return 0;
	float expProb = L_SHORT*exp(-L_SHORT*z);
	float normalizer = 1/(1-exp(-L_SHORT*trueZ));
	return expProb*normalizer;
}

// eq 6.10 from the book
float babs_slam::pMax(float z) {
	if (babs_slam::compareFloats(z, MAX_LIDAR_RANGE))
		return 1;
	return 0;
}

// eq 6.11 from the book
float babs_slam::pRand(float z) {
	if (z<0 || z>MAX_LIDAR_RANGE)
		return 0;
	return 1/MAX_LIDAR_RANGE;
}

bool babs_slam::compareFloats(float a, float b) {
	if (fabs(a-b) < 0.001)
		return true;
	return false;
}

void babs_slam::encoder_callback(const nav_msgs::Odometry& odom_value){

	double x = odom_value.pose.pose.position.x;
	double y = odom_value.pose.pose.position.y;

	//TODO do something with the odom values
	ROS_INFO("x,y from odom: %f, %f", x,y);

}
void babs_slam::imu_callback(const std_msgs::Float64& message_holder){

}
void babs_slam::gps_callback(const std_msgs::Float64& message_holder){

}
void babs_slam::lidar_callback(const sensor_msgs::LaserScan& laser_scan){

	float angle_min = laser_scan.angle_min;
	float angle_max = laser_scan.angle_max;
	float angle_increment = laser_scan.angle_increment;
	float time_increment = laser_scan.time_increment;
	float scan_time = laser_scan.scan_time;
	float range_min = laser_scan.range_min;
	float range_max = laser_scan.range_max;
	//laser_scan.ranges;
	//laser_scan.intensities;

	ROS_INFO("angle_min = %f", angle_min);
	ROS_INFO("angle_max = %f", angle_max);
	ROS_INFO("angle_increment = %f", angle_increment);
	ROS_INFO("time_increment = %f", time_increment);
	ROS_INFO("scan_time = %f", scan_time);
	ROS_INFO("range_min = %f", range_min);
	ROS_INFO("range_max = %f", range_max);



}


int main(int argc, char** argv) 
{


	ros::init(argc, argv, "babs_slam");

	ros::NodeHandle nh_;

	babs_slam babs(&nh_);

	ROS_INFO("initializing subscribers");
	ros::Subscriber encoder_listener= nh_.subscribe("/odom",1,babs_slam::encoder_callback);
	ros::Subscriber imu_listener= nh_.subscribe("imu_topic",1,babs_slam::imu_callback);
	ros::Subscriber gps_listener= nh_.subscribe("gps_topic",1,babs_slam::gps_callback);
	ros::Subscriber lidar_listener= nh_.subscribe("/scan",1,babs_slam::lidar_callback);

	/*
	// Sensor model testing code
	for (float i=0; i<11; i+=0.05) {
		float phit = babs.pHit(i,5);
		float pshort = babs.pShort(i, 5);
		float pmax = babs.pMax(i);
		float prand = babs.pRand(i);
		ROS_INFO("i=%f, phit=%f, pshort=%f, pmax=%f, prand=%f",i,phit, pshort,pmax,prand);
	}
	*/
    
    ros::spin();
    return 0;

} 
