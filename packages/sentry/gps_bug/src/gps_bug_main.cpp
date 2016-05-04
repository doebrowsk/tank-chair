#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <traj_builder/traj_builder.h>
#include <algorithm>

double dt = 0.02;
sensor_msgs::NavSatFix last_gps;
sensor_msgs::LaserScan last_scan;
sensor_msgs::LaserScan real_scan;
TrajBuilder trajBuilder; 
ros::Publisher twist_commander;

void gpscb(const sensor_msgs::NavSatFix& message_holder){
  last_gps = message_holder;
}

void lidarcb(const sensor_msgs::LaserScan& message_holder){
	last_scan = message_holder;
}
void reallidarcb(const sensor_msgs::LaserScan& message_holder){
	real_scan = message_holder;
}

void filter_angle_offset(float &angle, sensor_msgs::NavSatFix start, sensor_msgs::NavSatFix end){
	//ROS_INFO("Offset angle before filter:  %f", angle);

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
	//ROS_INFO("New offset angle after filter: %f", angle);
}

void move(float x, float y){

	std::vector<nav_msgs::Odometry> vec_of_states;
	ros::Rate loop(1/dt);

	geometry_msgs::PoseStamped start_pose;
	geometry_msgs::PoseStamped end_pose;
	

	start_pose.pose.position.x = 0.0;
    start_pose.pose.position.y = 0.0;
    start_pose.pose.position.z = 0.0;

    end_pose.pose.position.x = x;
    end_pose.pose.position.y = y;
    end_pose.pose.position.z = 0.0;

    trajBuilder.build_point_and_go_traj(start_pose, end_pose, vec_of_states);

	for (int i = 0; i < vec_of_states.size(); i++) {
        
        twist_commander.publish(vec_of_states[i].twist.twist); 

        loop.sleep(); 
    }

    ros::spinOnce();

}

void move_and_calibrate(float x, float y, float &angle){

	ros::spinOnce();
	sensor_msgs::NavSatFix old = last_gps;
	move(x,y);
	ros::spinOnce();
	filter_angle_offset(angle,old,last_gps);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_bug");
    ros::NodeHandle nh;
    twist_commander = nh.advertise<geometry_msgs::Twist>("gps_bug/cmd_vel", 1);
    ros::Rate looprate(1 / dt);

    ros::Publisher des_state_publisher = nh.advertise<nav_msgs::Odometry>("/desState", 1);
    ros::Publisher des_psi_publisher = nh.advertise<std_msgs::Float64>("/desPsi", 1);

    ros::Subscriber gps_sub = nh.subscribe("gps_fix_psr",1,gpscb);
    ros::Subscriber lidar_sub = nh.subscribe("gps_bug/cspace_scan",1,lidarcb);
    ros::Subscriber lidar_sub2 = nh.subscribe("scan",1,reallidarcb);
    sensor_msgs::NavSatFix goal;
    goal.latitude = 41.501918;
    goal.longitude = -81.608021;
    //last_gps.latitude = 0;
    //last_gps.longitude = 0;
    float gps_angle= 0.0; //assume the robot is facing east at first

    
    trajBuilder.set_dt(dt);
    trajBuilder.set_alpha_max(0.5);

    //CALIBRATION STUFF....................................................................................
    
 //    ros::spinOnce();
 //    // set initial gps location
 //    sensor_msgs::NavSatFix start_gps = last_gps;
 //    

 //    geometry_msgs::Twist g_halt_twist;
	// nav_msgs::Odometry g_end_state;
	// nav_msgs::Odometry g_start_state;
	// geometry_msgs::PoseStamped g_start_pose;
	// geometry_msgs::PoseStamped g_end_pose;

	// double psi_start = 0.0;
 //    double psi_end = 0.0; //3.0;
 //    g_start_state.pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_start);
 //    g_end_state = g_start_state;
 //    g_end_state.pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_end);

 //    g_start_pose.pose.position.x = 0.0;
 //    g_start_pose.pose.position.y = 0.0;
 //    g_start_pose.pose.position.z = 0.0;
 //    g_start_pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_start);
 //    g_end_pose = g_start_pose; //includes copying over twist with all zeros
 //    //don't really care about orientation, since this will follow from 
 //    // point-and-go trajectory; 
 //    g_end_pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_end);
 //    g_end_pose.pose.position.x = 5.0; //set goal coordinates
 //    g_end_pose.pose.position.y = 0.0; //-4.0;

 //    double des_psi;
 //    std_msgs::Float64 psi_msg;
 //    std::vector<nav_msgs::Odometry> vec_of_states;
 //    //trajBuilder.build_triangular_spin_traj(g_start_pose,g_end_pose,vec_of_states);
 //    //trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, vec_of_states);

 //    nav_msgs::Odometry des_state;
 //    nav_msgs::Odometry last_state;
 //    geometry_msgs::PoseStamped last_pose;

    
 //    ROS_INFO("calibrating heading according to GPS");
 //    trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, vec_of_states);

 //    for (int i = 0; i < vec_of_states.size(); i++) {
 //        des_state = vec_of_states[i];
 //        des_state.header.stamp = ros::Time::now();
 //        des_state_publisher.publish(des_state);
 //        des_psi = trajBuilder.convertPlanarQuat2Psi(des_state.pose.pose.orientation);
 //        psi_msg.data = des_psi;
 //        des_psi_publisher.publish(psi_msg);
 //        twist_commander.publish(des_state.twist.twist); //FOR OPEN-LOOP CTL ONLY!

 //        looprate.sleep(); //sleep for defined sample period, then do loop again
 //    }

 //    ros::spinOnce();
 //    // calibrate according to first move
 //    odom_offset = atan2(last_gps.latitude-start_gps.latitude,last_gps.longitude-start_gps.longitude);

 //    // start second calibration
 //    start_gps = last_gps;

 //    last_state = vec_of_states.back();
 //    last_pose.header = last_state.header;
 //    last_pose.pose = last_state.pose.pose;
 //    trajBuilder.build_point_and_go_traj(last_pose, g_start_pose, vec_of_states);
 //    for (int i = 0; i < vec_of_states.size(); i++) {
 //        des_state = vec_of_states[i];
 //        des_state.header.stamp = ros::Time::now();
 //        des_state_publisher.publish(des_state);
 //        des_psi = trajBuilder.convertPlanarQuat2Psi(des_state.pose.pose.orientation);
 //        psi_msg.data = des_psi;
 //        des_psi_publisher.publish(psi_msg);
 //        twist_commander.publish(des_state.twist.twist); //FOR OPEN-LOOP CTL ONLY!
 //        looprate.sleep(); //sleep for defined sample period, then do loop again
 //    }
 //    last_state = vec_of_states.back();
 //    g_start_pose.header = last_state.header;
 //    g_start_pose.pose = last_state.pose.pose;

 //    // compare first calibration phase to second
 //    ros::spinOnce();
 //    // load initial angle offset calculated, and the second move's gps points
 //    filter_angle_offset(odom_offset, start_gps, last_gps);
 //    start_gps = last_gps;

    // odom_offset now contains calibrated offset
    
    //DONE CALIBRATING............................................................................................................

    // run main routine

    //if path to goal is clear, head straight to goal, calib gps along the way

    //if path is blocked, follow with goal points with the best derivative
    int motion_state = 0;//0 = free 1 = left 2 = right
    bool goalpointfound;
    goalpointfound= true;
    bool done = false;
    while(!done){
    	
    	ros::spinOnce();
    	// ROS_INFO("has found goal: %s", goalpointfound.toString());
    	if(goalpointfound) {
    		ROS_INFO("Has found goal: TRUE");
    	} else {
    		ROS_INFO("Has found goal: FALSE");
    	}

        ROS_INFO("current angle: %f",gps_angle);
    	//rotate towards goal
        if (goalpointfound){
        	ROS_INFO("turning towards goal");
        	float rot_ang;
        	rot_ang = trajBuilder.min_dang(atan2(goal.latitude - last_gps.latitude,goal.longitude - last_gps.longitude)-gps_angle);
            ROS_INFO("rot %f", rot_ang);
        	move(cos (rot_ang)*0.01, sin(rot_ang)*0.01);
        	gps_angle+=rot_ang;
        	gps_angle = trajBuilder.min_dang(gps_angle);
            //ROS_INFO("initial rotate");
            
        }


    	ros::spinOnce();
        int midpoint = (last_scan.angle_max-last_scan.angle_min)/last_scan.angle_increment/2.0;
        if (last_scan.ranges[midpoint]<0.5){
        	// //run away from all points
        	// float xdir = 0;
        	// float ydir = 0;
        	// for (int i = 0; i < midpoint*2.0; i++){
        	// 	xdir -= cos(i*last_scan.angle_increment)/real_scan.ranges[i];
        	// 	ydir -= sin(i*last_scan.angle_increment)/real_scan.ranges[i];
        	// }
        	// float dir = atan2(ydir,xdir);
        	gps_angle+= 3.14159;
        	move_and_calibrate(-3,0,gps_angle);
        	ROS_INFO("run away");
        }
    	else if (last_scan.ranges[midpoint]>7.0){
    		move_and_calibrate(5,0,gps_angle);
    		//move forward 5 meters
    		//calibrate gps while doing that
    		motion_state=0;
            goalpointfound = true;
            ROS_INFO("easy case");
            
    	}
    	// cant move forward, need to try turning
    	else{
    		// if currently turning left
    		// or it is most favorable to turn left, and we were moving forward before
    		if (motion_state == 1 || (last_scan.ranges[midpoint-1]>last_scan.ranges[midpoint+1]&&motion_state==0)){
    			motion_state =1;
    			//turn left
    			ROS_INFO("turn left");
    			goalpointfound = false;

    			// 
    			for (int i = midpoint-1; i > 0; i --){
    				//look for a discontinuity
    				if (last_scan.ranges[i]-last_scan.ranges[i+1]>1.0|| last_scan.ranges[i]>6.0){
    					float laser_scan_angle = last_scan.angle_min + last_scan.angle_increment*i;
    					gps_angle+=laser_scan_angle;
                        float dist = std::min(5.0f,last_scan.ranges[i+1]);
    					move_and_calibrate(cos(laser_scan_angle)*dist-3,sin(laser_scan_angle)*dist+1.5,gps_angle);
    					//goalpoint = that point - 1 meter;
    					ROS_INFO("goal found");
    					goalpointfound = true;
    					break;
    				}
    				if (last_scan.ranges[i]-last_scan.ranges[i+1]<-1.0){
    					float laser_scan_angle = last_scan.angle_min + last_scan.angle_increment*i;
    					gps_angle+=laser_scan_angle;
                        float dist = std::min(5.0f,last_scan.ranges[i+1]);
    					move_and_calibrate(cos(laser_scan_angle)*dist-3,sin(laser_scan_angle)*dist,gps_angle);
    					//goalpoint = that point - 1 meter;
    					goalpointfound = true;
    					ROS_INFO("goal found");
    					break;
    				}
    			}
    			//if no solution found, turn 90 degrees
    			if (!goalpointfound){

    				move(cos (1.5708)*0.01, sin(1.5708)*0.01);
                    gps_angle+=1.5708;
                    ROS_INFO("hard left");
                    goalpointfound=false;
    			}
    		}else{
    			ROS_INFO("turn right");
    			motion_state=2;
    			bool goalpointfound = false;
    			for (int i = midpoint+1; i < midpoint*2-1; i++){
    				if (last_scan.ranges[i]-last_scan.ranges[i-1]>1.0|| last_scan.ranges[i]>6.0){
    					float laser_scan_angle = last_scan.angle_min + last_scan.angle_increment*i;
    					gps_angle+=laser_scan_angle;
                        float dist = std::min(5.0f,last_scan.ranges[i-1]);
    					move_and_calibrate(cos(laser_scan_angle)*dist-3,sin(laser_scan_angle)*dist-1.5,gps_angle);
    					ROS_INFO("goal found");
    					goalpointfound = true;
    					break;
    				}
    				if (last_scan.ranges[i]-last_scan.ranges[i-1]<-1.0){
    					float laser_scan_angle = last_scan.angle_min + last_scan.angle_increment*i;
    					gps_angle+=laser_scan_angle;
                        float dist = std::min(5.0f,last_scan.ranges[i-1]);
    					move_and_calibrate(cos(laser_scan_angle)*dist-3,sin(laser_scan_angle)*dist,gps_angle);
    					ROS_INFO("goal found");
    					goalpointfound = true;
    					break;
    				}

    			}
    			//turn 90 degrees
    			if (!goalpointfound){
    				move(cos (-1.5708)*0.01, sin(-1.5708)*0.01);
                    gps_angle-=1.5708;
                    ROS_INFO("hard right");
                    goalpointfound = false;
    			}
    		}
    	}

	    if(sqrt(pow(goal.latitude - last_gps.latitude,2)+pow(goal.longitude - last_gps.longitude,2))<0.00009){
	    	done = true;
	    }
	}



}