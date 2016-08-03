#include <ros/ros.h> 
#include <std_msgs/Int16.h> 
#include <std_msgs/Float32MultiArray.h> 
#include <std_msgs/Float32.h> 

#include <sensor_msgs/LaserScan.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h> // point cloud?
#include <pcl/point_types.h>
#include <math.h>       /* sin */

#define PI 3.14159265

double angle;
pcl::PointCloud<pcl::PointXYZ> cloud;

void myCallback(const std_msgs::Int16& message_holder) 
{ 
    // The real work is done in this callback function 
    // it wakes up every time a new message is published on "topic1" 
    // Since this function is prompted by a message event, 
    //it does not consume CPU time polling for new data 
    // the ROS_INFO() function is like a printf() function, except 
    // it publishes its output to the default rosout topic, which prevents 
    // slowing down this function for display calls, and it makes the 
    // data available for viewing and logging purposes 

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

void myCallback2(const sensor_msgs::LaserScan::ConstPtr& scan) 
{ 
    // the real work is done in this callback function 
    // it wakes up every time a new message is published on "topic1" 
    // Since this function is prompted by a message event, 
    //it does not consume CPU time polling for new data 
    // the ROS_INFO() function is like a printf() function, except 
    // it publishes its output to the default rosout topic, which prevents 
    // slowing down this function for display calls, and it makes the 
    // data available for viewing and logging purposes 
    //ROS_INFO_STREAM("scanis" << scan->ranges[1]); 

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
    //TODO: name this node 
    ros::init(argc,argv,"hokuyo_pcl"); 
    ros::NodeHandle nh; 

    ros::Subscriber my_subscriber_object = nh.subscribe("/dynamixel_motor1_ang", 1, myCallback); 
    ros::Subscriber my_subscriber_object2 = nh.subscribe("/scan", 1, myCallback2); 

    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_cloud_display", 1);

    while (nh.ok())
    {
        //cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = "something";

        //if(angle < 2 && cloud.size() > 100000)
        //cloud.clear();

        pubCloud.publish(cloud);
        ros::Duration(0.05).sleep();
        ros::spinOnce ();
    }

    //ros::spin(); //this is essentially a "while(1)" statement, except it 
    // forces refreshing wakeups upon new data arrival 
    // main program essentially hangs here, but it must stay alive to keep the callback function alive 
    //return 0; // should never get here, unless roscore dies 
} 
