#include <ros/ros.h> 
#include <std_msgs/Int16.h> 
#include <std_msgs/Float32MultiArray.h> 
#include <std_msgs/Float32.h> 

#include <sensor_msgs/LaserScan.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h> 
#include <pcl/point_types.h>
#include <math.h>       /* sin */

#define PI 3.14159265

double angle;
pcl::PointCloud<pcl::PointXYZ> cloud;

// Gets the angle of the current motor position
void hokuyoMotorCallback(const std_msgs::Int16& message_holder) 
{ 
    if (message_holder.data>1100) 
    {
        ROS_WARN("noise");
    }
    else
    {
        angle = message_holder.data * 90 / 1024;
    }
    ROS_INFO_STREAM("received value is:" << angle); 
    // Really could do something interesting here with the received data...but all we do is print it 
} 

// Transforms the scan based on the motor angle
void hokuyoCallback(const sensor_msgs::LaserScan::ConstPtr& scan) 
{ 
    double angle2;
    double x,y,z;
    for(int i = 0; i < 512; i++)
    {
        angle2 = i*180/512;
        
        // TODO: figure out what .082 is and replace with a #define
        y = sin(angle * PI / 180) * sin(angle2 * PI / 180) * scan->ranges[i] + .082 * cos(angle * PI / 180);;
        x = sin(angle * PI / 180) * cos(angle2 * PI / 180) * scan->ranges[i];
        z = cos(angle * PI / 180) * sin(angle2 * PI / 180) * scan->ranges[i] * -1.0 + .082 * cos(angle * PI / 180);

        cloud.push_back (pcl::PointXYZ (x, y, z));
    }
    // Really could do something interesting here with the received data...but all we do is print it 
} 

int main(int argc, char **argv) 
{ 
    ros::init(argc,argv,"hokuyo_pcl"); 
    ros::NodeHandle nh; 

    ros::Subscriber my_subscriber_object = nh.subscribe("/dynamixel_motor1_ang", 1, hokuyoMotorCallback); 
    ros::Subscriber my_subscriber_object2 = nh.subscribe("/scan", 1, hokuyoCallback); 

    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_cloud_display", 1);

    while (nh.ok())
    {
        //cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = "wobbler_frame";

        //if(angle < 2 && cloud.size() > 100000)
        //cloud.clear();

        pubCloud.publish(cloud);
        ros::Duration(0.05).sleep();
        ros::spinOnce ();
    }

    //ros::spin(); 
    //return 0; 
} 
