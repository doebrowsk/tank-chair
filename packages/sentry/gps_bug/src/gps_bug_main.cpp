#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

double dt = 0.02;
nav_msgs::NavSatFix last_gps;

void gpscb(const nav_msgs::NavSatFix& message_holder){
  last_gps = message_holder;
}

void filter_angle_offset(float &angle, nav_msgs::NavSatFix start, nav_msgs::NavSatFix end){
	// //this is some sort of filter i guess
	// float dist = sqrt(pow(start.latitude-end.latitude,2)+pow(start.longitude-end.longitude,2));
	// float xcomp = cos(angle)*5 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "des_state_publisher");
    ros::NodeHandle nh;
    ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Rate looprate(1 / dt);

    ros::Subscriber gps_sub = nh.subscribe("gps_fix_psr",1,gpscb);

    //CALIBRATION STUFF....................................................................................
    float odom_offset = 0.0;
    ros::spinOnce();
    nav_msgs::NavSatFix start_gps = last_gps;
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
    odom_offset = atan2(last_gps.latitude-start_gps.latitude,last_gps.longitude-start_gps.longitude);
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
    
    //DONE CALIBRATING............................................................................................................



}