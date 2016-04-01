// expects client to give an integer corresponding to a timer count, in seconds
// server counts up to this value, provides feedback, and can be cancelled any time
// re-use the existing action message, although not all fields are needed
// use request "input" field for timer setting input, 
// value of "fdbk" will be set to the current time (count-down value)
// "output" field will contain the final value when the server completes the goal request

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../sentry_urdf/action/path.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (path) and appended name (Action)
#include<sentry_urdf/pathAction.h>
#include <sys/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <gazebo_msgs/ModelStates.h>

int g_count = 0;
bool g_count_failure = false;
geometry_msgs::Pose g_current_pose; //track current pose
geometry_msgs::Twist g_current_twist;


class SentryActionServer {
private:

	ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

	//some tunable constants, global
	static const double g_move_speed = 1.0; // set forward speed to this value, e.g. 1m/s
	static const double g_spin_speed = 1.0; // set yaw rate to this value, e.g. 1 rad/s
	static const double g_sample_dt = 0.01;
	static const double g_dist_tol = 0.01; // 1cm
	static const double g_angle_tol = 0.035; //about 2 degrees;
	static const double g_linear_motion_tol = 0.01; //in m/s
	static const double g_angular_motion_tol = 0.01; //in rad/s

	// this class will own a "SimpleActionServer" called "as_".
	// it will communicate using messages defined in path.action
	// the type "pathAction" is auto-generated from our name "path" and generic name "Action"
	actionlib::SimpleActionServer<sentry_urdf::pathAction> as_;

	// here are some message types to communicate with our client(s)
	sentry_urdf::pathGoal goal_; // goal message, received from client
	sentry_urdf::pathResult result_; // put results here, to be sent back to the client when done w/ goal
	sentry_urdf::pathFeedback feedback_; // for feedback
	//  use: as_.publishFeedback(feedback_); to send incremental feedback to the client
	int countdown_val_;

	geometry_msgs::Twist twist_cmd_;
	ros::Publisher cmd_publisher_; // sends twist commands to cmd_vel topic
	ros::Subscriber state_sub;

public:


	SentryActionServer(); //define the body of the constructor outside of class definition

	~SentryActionServer(void) {
	}
	// Action Interface
	void executeCB(const actionlib::SimpleActionServer<sentry_urdf::pathAction>::GoalConstPtr& goal);

	static void model_state_CB(const gazebo_msgs::ModelStates& model_states);


	// here are a few useful utility functions:
	static double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
	static void get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose, double &dist, double &heading);

	static double sgn(double x);
	static double min_spin(double spin_angle);
	static geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);
	void do_halt();
	bool do_move(double distance);
	bool do_spin(double yaw_desired);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, 
// which is a member method of our class SentryActionServer.
// Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB function takes one argument
// The final argument,  "false", says don't start the server yet.  (We'll do this in the constructor)

SentryActionServer::SentryActionServer() :
		as_(nh_, "sentry_action", boost::bind(&SentryActionServer::executeCB, this, _1),false)
// in the above initialization, we name the server "sentry_action"
//  clients will need to refer to this name to connect with this server
{

	ROS_INFO("in constructor of SentryActionServer...");

	//init publisher
	cmd_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true); // commands the robot!
	//initialize the twist command components, all to zero
	twist_cmd_.linear.x = 0.0;
	twist_cmd_.linear.y = 0.0;
	twist_cmd_.linear.z = 0.0;
	twist_cmd_.angular.x = 0.0;
	twist_cmd_.angular.y = 0.0;
	twist_cmd_.angular.z = 0.0;

	as_.start(); //start the server running

	state_sub = nh_.subscribe("gazebo/model_states",1, SentryActionServer::model_state_CB);

}

void SentryActionServer::model_state_CB(const gazebo_msgs::ModelStates& model_states)
{

	int n_models = model_states.name.size();
	int imodel;
	bool found_name=false;
	for (imodel=0;imodel<n_models;imodel++) {
		std::string model_name(model_states.name[imodel]);
		if (model_name.compare("sentry")==0) {
			found_name=true;
			break;
		}
	}
	if(found_name) {
		//Sentry pose
		g_current_pose = model_states.pose[imodel];
		//Sentry motion
		g_current_twist = model_states.twist[imodel];
	}
	else
	{
		ROS_WARN("state of Sentry model not found");
	}
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <sentry_urdf::pathAction> customizes the simple action server to use our own "action" message
// defined in our package, "sentry_urdf", in the subdirectory "action", called "path.action"
// The name "path" is prepended to other message types created automatically during compilation.
// e.g.,  "pathAction" is auto-generated from (our) base name "path" and generic name "Action"
void SentryActionServer::executeCB(const actionlib::SimpleActionServer<sentry_urdf::pathAction>::GoalConstPtr& goal) {

	ROS_INFO("in executeCB");

	double yaw_desired, travel_distance;
	geometry_msgs::Pose pose_desired;
	int npts = goal->path.poses.size();
	ROS_INFO("received path request with %d poses",npts);

	//a rotation goal
	if (convertPlanarQuat2Phi(goal->path.poses[0].pose.orientation) != 0) {
		ROS_INFO("Starting to rotate");
		ros::Rate loop_timer(1/g_sample_dt);

		//rotate until lidar alarm turns off and cancels the goal
		while(!as_.isPreemptRequested()) {
			twist_cmd_.angular.z = g_spin_speed;
			cmd_publisher_.publish(twist_cmd_);
			loop_timer.sleep();
		}
		ROS_WARN("rotation goal cancelled!");

		twist_cmd_.angular.z = 0.0;
		cmd_publisher_.publish(twist_cmd_);

		//Send location back when goal completed, used for planning new square path
		result_.output = -1;
		result_.final_pose = g_current_pose;

		as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
		return; // done with callback

	}
	else {
		//a path goal
		for (int i=0;i<npts;i++) { //visit each subgoal

			pose_desired = goal->path.poses[i].pose; //get next pose from vector of poses

			ROS_INFO("pose %d", i);
			ROS_INFO("current (x,y) = (%f, %f)",g_current_pose.position.x,g_current_pose.position.y);
			ROS_INFO("desired (x,y) = (%f, %f)",pose_desired.position.x,pose_desired.position.y);

			get_yaw_and_dist(g_current_pose, pose_desired, travel_distance, yaw_desired);

			ROS_INFO("travel_distance = %f", travel_distance);
			ROS_INFO("yaw_desired = %f", yaw_desired);

			bool aborted = do_spin(yaw_desired);

			ROS_INFO("done spinnin");
			aborted  = do_move(travel_distance);

			if (aborted) {
				ROS_WARN("goal cancelled! (move)");

				result_.output = -1;
				result_.final_pose = g_current_pose;
				as_.setAborted(result_);
				return;
			}

			ROS_INFO("done movin--------------------------------------------------------------------------------");

			//if here, then goal is still valid; provide some feedback
			feedback_.fdbk = i; // populate feedback message with pose that was just acheived
			feedback_.last_pose = g_current_pose;
			as_.publishFeedback(feedback_); // send feedback to the action client that requested this goal
		}
	}

	//if we survive to here, then the goal was successfully accomplished; inform the client
	do_halt();

	result_.output = 42; //set to 42 because why not
	result_.final_pose = g_current_pose;
	as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
}

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
bool SentryActionServer::do_spin(double yaw_desired) {

	yaw_desired = min_spin(yaw_desired);

	double yaw_current = min_spin(convertPlanarQuat2Phi(g_current_pose.orientation));

	ROS_INFO("yaw_current = %f", yaw_current);

	double spin_angle = yaw_desired - yaw_current; // spin this much
	double spin_direction = sgn(min_spin(spin_angle));// but what if this angle is > pi?  then go the other way

	double og_diff = fabs(yaw_desired - yaw_current);
	double current_diff = og_diff;

	ros::Rate loop_timer(1/g_sample_dt);

	while(current_diff > g_angle_tol) {

		//so that the turn will slow down as we approach the desired angle, with a min rotation speed
		twist_cmd_.angular.z = spin_direction*g_spin_speed*current_diff/og_diff + 10*spin_direction*g_angular_motion_tol;
		cmd_publisher_.publish(twist_cmd_);
		loop_timer.sleep();
		yaw_current = min_spin(convertPlanarQuat2Phi(g_current_pose.orientation));
		current_diff = fabs(yaw_desired - yaw_current);
	}

	do_halt();
	return false;
}

//a function to move forward by a specified distance (in meters), then halt
bool SentryActionServer::do_move(double distance) { // always assumes robot is already oriented properly

	double xStart = g_current_pose.position.x;
	double yStart = g_current_pose.position.y;

	ros::Rate loop_timer(1/g_sample_dt);

	double dist_traveled = 0.0;
	//stop when we've gone the distance
	while(dist_traveled < distance) {

		if(as_.isPreemptRequested()) {
			do_halt();
			return true; //indicate that goal was aborted
		}

		//so that the motion will slow down as we approach the desired distance, with a min approach speed
		twist_cmd_.linear.x = sgn(distance)*g_move_speed*(1 - dist_traveled/distance) + 10*sgn(distance)*g_linear_motion_tol;
		cmd_publisher_.publish(twist_cmd_);
		loop_timer.sleep();
		dist_traveled = sqrt(pow(g_current_pose.position.x - xStart, 2) + pow(g_current_pose.position.y - yStart, 2));
	}

	do_halt();
	return false;
}

void SentryActionServer::do_halt() {

	ROS_INFO("halting");

	double linearMotion = g_current_twist.linear.x;
	double angularMotion = g_current_twist.angular.z;

	ros::Rate loop_timer(1/g_sample_dt);

	while(linearMotion > g_linear_motion_tol || angularMotion > g_angular_motion_tol) {

		//slam on the brakes
		twist_cmd_.angular.z = -10*angularMotion;
		twist_cmd_.linear.x = -10*linearMotion;

		cmd_publisher_.publish(twist_cmd_);
		loop_timer.sleep();
		linearMotion = g_current_twist.linear.x;
		angularMotion = g_current_twist.angular.z;
	}

	twist_cmd_.angular.z = 0.0;
	twist_cmd_.linear.x = 0.0;
	cmd_publisher_.publish(twist_cmd_);

	ROS_INFO("halted");
}

//signum function: strip off and return the sign of the argument
double SentryActionServer::sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double SentryActionServer::min_spin(double spin_angle) {
        if (spin_angle>M_PI) {
            spin_angle -= 2.0*M_PI;}
        if (spin_angle< -M_PI) {
            spin_angle += 2.0*M_PI;}
         return spin_angle;
}

// a useful conversion function: from quaternion to yaw
double SentryActionServer::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
	double quat_z = quaternion.z;
	double quat_w = quaternion.w;
	double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
	return phi;
}

//and the other direction:
geometry_msgs::Quaternion SentryActionServer::convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

void SentryActionServer::get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading) {

	double dx = goal_pose.position.x - current_pose.position.x;
	double dy = goal_pose.position.y - current_pose.position.y;

	dist = sqrt(pow(dx, 2) + pow(dy,2));
	heading = atan2(dy,dx);
}

int main(int argc, char** argv) {

	//define initial position to be 0
	g_current_pose.position.x = 0.0;
	g_current_pose.position.y = 0.0;
	g_current_pose.position.z = 0.0;

	// define initial heading to be "0"
	g_current_pose.orientation.x = 0.0;
	g_current_pose.orientation.y = 0.0;
	g_current_pose.orientation.z = 0.0;
	g_current_pose.orientation.w = 1.0;

	ros::init(argc, argv, "sentry_action_server_node"); // name this node

	ROS_INFO("instantiating the sentry_action_server: ");

	SentryActionServer as_object; // create an instance of the class "SentryActionServer"

	ROS_INFO("going into spin");
	// from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
	// you will see 5 new topics under sentry_action: cancel, feedback, goal, result, status
	ros::spin();

	return 0;
}

