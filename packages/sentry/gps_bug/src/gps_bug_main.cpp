#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <traj_builder/traj_builder.h>

double dt = 0.02;
sensor_msgs::NavSatFix last_gps;

void gpscb(const sensor_msgs::NavSatFix& message_holder){
  last_gps = message_holder;
}

void filter_angle_offset(float &angle, sensor_msgs::NavSatFix start, sensor_msgs::NavSatFix end){
	ROS_INFO("Offset angle before filter:  %f", angle);

	// linear distance covered in second gps calibration
	// float dist = sqrt(pow(start.latitude-end.latitude,2)+pow(start.longitude-end.longitude,2));

	// find nominal values of distance covered based on current offset angle
	// nominal x distance component
	float nom_x_comp = cos(angle);
	// nominal y dsitance
	float nom_y_comp = sin(angle);

	// find stated gps distance components
	// first get gps angle of motion
	float gps_angle = atan2((end.latitude - start.latitude), (end.longitude - start.longitude));
	// gps x distance
	float gps_x_comp = cos(gps_angle);
	// gps y distance
	float gps_y_comp = sin(gps_angle);

	// average x and y components
	// prefer nominal component over gps: 0.7 weight vs 0.3 for adjustment
	float x_comp = nom_x_comp * 0.7 + gps_x_comp * 0.3;
	float y_comp = nom_y_comp * 0.7 + gps_y_comp * 0.3;

	// compute a new offset angle
	angle = atan2(y_comp, x_comp);
	ROS_INFO("New offset angle after filter: %f", angle);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_bug");
    ros::NodeHandle nh;
    ros::Publisher twist_commander = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Rate looprate(1 / dt);

    ros::Publisher des_state_publisher = nh.advertise<nav_msgs::Odometry>("/desState", 1);
    ros::Publisher des_psi_publisher = nh.advertise<std_msgs::Float64>("/desPsi", 1);

    ros::Subscriber gps_sub = nh.subscribe("gps_fix_psr",1,gpscb);

    //CALIBRATION STUFF....................................................................................
    float odom_offset = 0.0;
    ros::spinOnce();
    // set initial gps location
    sensor_msgs::NavSatFix start_gps = last_gps;
    TrajBuilder trajBuilder; 
    trajBuilder.set_dt(dt);
    trajBuilder.set_alpha_max(0.6);

    geometry_msgs::Twist g_halt_twist;
	nav_msgs::Odometry g_end_state;
	nav_msgs::Odometry g_start_state;
	geometry_msgs::PoseStamped g_start_pose;
	geometry_msgs::PoseStamped g_end_pose;

	double psi_start = 0.0;
    double psi_end = 0.0; //3.0;
    g_start_state.pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_start);
    g_end_state = g_start_state;
    g_end_state.pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_end);

    g_start_pose.pose.position.x = 0.0;
    g_start_pose.pose.position.y = 0.0;
    g_start_pose.pose.position.z = 0.0;
    g_start_pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_start);
    g_end_pose = g_start_pose; //includes copying over twist with all zeros
    //don't really care about orientation, since this will follow from 
    // point-and-go trajectory; 
    g_end_pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_end);
    g_end_pose.pose.position.x = 5.0; //set goal coordinates
    g_end_pose.pose.position.y = 0.0; //-4.0;

    double des_psi;
    std_msgs::Float64 psi_msg;
    std::vector<nav_msgs::Odometry> vec_of_states;
    //trajBuilder.build_triangular_spin_traj(g_start_pose,g_end_pose,vec_of_states);
    //trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, vec_of_states);

    nav_msgs::Odometry des_state;
    nav_msgs::Odometry last_state;
    geometry_msgs::PoseStamped last_pose;

    
    ROS_INFO("calibrating heading according to GPS");
    trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, vec_of_states);

    for (int i = 0; i < vec_of_states.size(); i++) {
        des_state = vec_of_states[i];
        des_state.header.stamp = ros::Time::now();
        des_state_publisher.publish(des_state);
        des_psi = trajBuilder.convertPlanarQuat2Psi(des_state.pose.pose.orientation);
        psi_msg.data = des_psi;
        des_psi_publisher.publish(psi_msg);
        twist_commander.publish(des_state.twist.twist); //FOR OPEN-LOOP CTL ONLY!

        looprate.sleep(); //sleep for defined sample period, then do loop again
    }

    ros::spinOnce();
    // calibrate according to first move
    odom_offset = atan2(last_gps.latitude-start_gps.latitude,last_gps.longitude-start_gps.longitude);

    // start second calibration
    start_gps = last_gps;

    last_state = vec_of_states.back();
    last_pose.header = last_state.header;
    last_pose.pose = last_state.pose.pose;
    trajBuilder.build_point_and_go_traj(last_pose, g_start_pose, vec_of_states);
    for (int i = 0; i < vec_of_states.size(); i++) {
        des_state = vec_of_states[i];
        des_state.header.stamp = ros::Time::now();
        des_state_publisher.publish(des_state);
        des_psi = trajBuilder.convertPlanarQuat2Psi(des_state.pose.pose.orientation);
        psi_msg.data = des_psi;
        des_psi_publisher.publish(psi_msg);
        twist_commander.publish(des_state.twist.twist); //FOR OPEN-LOOP CTL ONLY!
        looprate.sleep(); //sleep for defined sample period, then do loop again
    }
    last_state = vec_of_states.back();
    g_start_pose.header = last_state.header;
    g_start_pose.pose = last_state.pose.pose;

    // compare first calibration phase to second
    ros::spinOnce();
    // load initial angle offset calculated, and the second move's gps points
    filter_angle_offset(odom_offset, start_gps, last_gps);

    // odom_offset now contains calibrated offset
    
    //DONE CALIBRATING............................................................................................................

    // run main routine
}