// sentry_client: works together with action server called "sentry_action_server_w_fdbk"
// in source: sentry_action_server_w_fdbk.cpp
// this code could be written using classes instead (e.g. like the corresponding server)
//  see: http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <sentry_urdf/pathAction.h> //reference action message in this package
#include <sys/time.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <std_msgs/Bool.h>


using namespace std;

bool g_goal_active = false; //some global vars for communication with callbacks
int g_result_output = -1;
int g_fdbk = -1;
bool g_lidar_alarm=false; // global var for lidar alarm
geometry_msgs::Pose g_last_known_pose; //track current pose


void alarmCallback(const std_msgs::Bool& alarm_msg)
{
	g_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
}

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
		const sentry_urdf::pathResultConstPtr& result) {

	ROS_INFO("doneCb: server responded with state [%s]", state.toString().c_str());
	g_last_known_pose = result->final_pose;
	g_result_output= result->output;
	g_goal_active=false;

	ROS_INFO("doneCb g_last_known_pose (x,y) = (%f,%f)", g_last_known_pose.position.x, g_last_known_pose.position.y);

}

//this function wakes up every time the action server has feedback updates for this client
// only the client that sent the current goal will get feedback info from the action server
void feedbackCb(const sentry_urdf::pathFeedbackConstPtr& fdbk_msg) {
	ROS_INFO("Feedback: pose %d complete",fdbk_msg->fdbk);

	g_last_known_pose = fdbk_msg->last_pose;
	g_fdbk = fdbk_msg->fdbk; //make status available to "main()"

	ROS_INFO("g_last_known_pose from feedback (x,y) = (%f,%f)", g_last_known_pose.position.x, g_last_known_pose.position.y);
}

// Called once when the goal becomes active; not necessary, but could be useful diagnostic
void activeCb()
{
	ROS_INFO("Goal just went active");
	g_goal_active=true; //let main() know that the server responded that this goal is in process
}

// a useful conversion function: from quaternion to yaw
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
	double quat_z = quaternion.z;
	double quat_w = quaternion.w;
	double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
	return phi;
}

//a function to consider periodicity and find min delta angle
double min_spin(double spin_angle) {
        if (spin_angle>M_PI) {
            spin_angle -= 2.0*M_PI;}
        if (spin_angle< -M_PI) {
            spin_angle += 2.0*M_PI;}
         return spin_angle;
}

int main(int argc, char** argv) {

	//assume we start at the origin
	geometry_msgs::Pose og_pose;
	og_pose.orientation.x = 0.0;
	og_pose.orientation.y = 0.0;
	og_pose.orientation.z = 0.0;
	og_pose.orientation.w = 1.0;
	og_pose.position.x = 0.0;
	og_pose.position.y = 0.0;
	og_pose.position.z = 0.0;
	g_last_known_pose = og_pose;

	timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	ROS_INFO("sentry_client main-----> %lld.%.9ld", (long long)ts.tv_sec, ts.tv_nsec);

	ros::init(argc, argv, "sentry_client_node"); // name this node
	ros::NodeHandle n;

    ros::Subscriber alarm_subscriber = n.subscribe("lidar_alarm",1,alarmCallback);

	// use the name of our server, which is: sentry_action (named in sentry_action_server_w_fdbk.cpp)
	// the "true" argument says that we want our new client to run as a separate thread (a good idea)
	actionlib::SimpleActionClient<sentry_urdf::pathAction> action_client("sentry_action", true);

	// attempt to connect to the server: need to put a test here, since client might launch before server
	ROS_INFO("attempting to connect to server: ");
	bool server_exists = action_client.waitForServer(ros::Duration(1.0)); // wait for up to 1 second
	// something odd in above: sometimes does not wait for specified seconds,
	//  but returns rapidly if server not running; so we'll do our own version
	while (!server_exists) { // keep trying until connected
		ROS_WARN("could not connect to server; retrying...");
		server_exists = action_client.waitForServer(ros::Duration(1.0)); // retry every 1 second
	}
	ROS_INFO("connected to action server");  // if here, then we connected to the server;

	ros::Rate timer(100); // 100Hz timer

	geometry_msgs::Pose pose_for_plan = g_last_known_pose;

	while(true) {

		while(g_goal_active) {
			ros::spinOnce();
			if (g_lidar_alarm) {
				ROS_INFO("Alarm triggered, cancelling path");

				sentry_urdf::pathGoal goal;
				geometry_msgs::PoseStamped pose_stamped;
				geometry_msgs::Pose pose;

				geometry_msgs::Pose oldPose = g_last_known_pose;
				action_client.cancelGoal();

				//wait until we know the pose has been updated
				while (oldPose.orientation.z == g_last_known_pose.orientation.z) {
					ROS_INFO("Waiting for orientation to update");
					ros::spinOnce();
					timer.sleep();
				}

				pose_for_plan = g_last_known_pose;

				pose.orientation.x = 0.0;
				pose.orientation.y = 0.0;
				pose.orientation.z = sqrt(2.0)/2;
				pose.orientation.w = sqrt(2.0)/2; //rotate 90 degrees
				pose_stamped.pose = pose;
				goal.path.poses.push_back(pose_stamped);
				action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

				while (g_lidar_alarm) {
					ros::spinOnce();
					timer.sleep();
				}
				ROS_INFO("Alarm off, cancelling rotation");

				action_client.cancelGoal(); //cancel when lidar alarm turns off

				break;
			}
			else {
				timer.sleep();
			}
		}

		//Commands to move in a square
		sentry_urdf::pathGoal goal;
		geometry_msgs::PoseStamped pose_stamped;
		geometry_msgs::Pose pose;


		pose.orientation.x = 0.0; //set these to zero to indicate that we do not care about them
		pose.orientation.y = 0.0; //(required orientations are derived from the positions)
		pose.orientation.z = 0.0;
		pose.orientation.w = 1.0;
		pose_stamped.pose = pose;

		double heading = min_spin(convertPlanarQuat2Phi(pose_for_plan.orientation));

		double dx = 3.0;
		double dy;

		if (heading < 0) {
			dy = 3.0;
		}
		else {
			dy = -3.0;
		}

		ROS_INFO("heading, dy = %f, %f", heading, dy);


		pose_stamped.pose.position.x = pose_for_plan.position.x;
		pose_stamped.pose.position.y = pose_for_plan.position.y + dy;;

		ROS_INFO("(x,y) = (%f,%f)", pose_stamped.pose.position.x,pose_stamped.pose.position.y);

		pose_stamped.pose.position.z = 0.0;
		goal.path.poses.push_back(pose_stamped);

		pose_stamped.pose.position.x = pose_for_plan.position.x + dx;
		pose_stamped.pose.position.y = pose_for_plan.position.y + dy;

		ROS_INFO("(x,y) = (%f,%f)", pose_stamped.pose.position.x,pose_stamped.pose.position.y);

		goal.path.poses.push_back(pose_stamped);

		pose_stamped.pose.position.x = pose_for_plan.position.x + dx;
		pose_stamped.pose.position.y = pose_for_plan.position.y + dy/3;

		ROS_INFO("(x,y) = (%f,%f)", pose_stamped.pose.position.x,pose_stamped.pose.position.y);

		goal.path.poses.push_back(pose_stamped);

		pose_stamped.pose.position.x = pose_for_plan.position.x;
		pose_stamped.pose.position.y = pose_for_plan.position.y + dy/3;

		ROS_INFO("(x,y) = (%f,%f)", pose_stamped.pose.position.x,pose_stamped.pose.position.y);

		goal.path.poses.push_back(pose_stamped);

		ROS_INFO("sending goal to client");


		//here are some options:
		//action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
		//action_client.sendGoal(goal,&doneCb); // send goal and specify a callback function
		//or, send goal and specify callbacks for "done", "active" and "feedback"
		action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

		//take a quick snooze, give the server time to verify that the goal is active
		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}

