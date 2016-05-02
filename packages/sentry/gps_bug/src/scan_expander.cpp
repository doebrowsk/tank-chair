#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

ros::Publisher scanpub;

void scancb(const sensor_msgs::LaserScan& message_holder){
	float radius = 1.3;//expansion of each scan
	sensor_msgs::LaserScan cspace_scan = message_holder;
	cspace_scan.range_min = 0;
	for (int scantofill = 0; scantofill < (message_holder.angle_max - message_holder.angle_min)/message_holder.angle_increment;scantofill++){
		for (int compare = 0; compare < (message_holder.angle_max - message_holder.angle_min)/message_holder.angle_increment;compare++){
			float distance = message_holder.ranges[compare];
			if (distance < radius){
				cspace_scan.ranges[scantofill]=0.0;
				continue;
			}
			float horizon = sqrt(pow(distance,2)-pow(radius,2));
			float scan_ang_size = acos(horizon/distance);
			float circle_ang_size = 90 - scan_ang_size;
			float scan_ang_diff = abs(scantofill-compare)/cspace_scan.angle_increment;
			float cspace_range = (distance-radius)*(1+1/cos(ang_diff));// distance to the line tangent to the circle
			cspace_range+= radius-cos(circle_ang_size*scan_ang_diff/scan_ang_size);
			if (cspace_range<cspace_scan.ranges[scantofill]){
				cspace_scan.ranges[scantofill]=cspace_range;
			}

		}
	}
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "config_space");
    ros::NodeHandle nh;
    scanpub = nh.advertise<sensor_msgs::LaserScan>("gps_bug/cspace_scan", 1);
    ros::Subscriber scansub = nh.subscribe("scan",1,scancb);
    ros::Spin();

}