#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <roboteq_msgs/Command.h>


ros::Publisher ch1, ch2;
float MAGIC_NUM = 0.833;
void myCallback(const geometry_msgs::Twist& message_holder) 
{
	float v1,v2;
	v2 += message_holder.angular.z/2.0;
	v1 -= message_holder.angular.z/2.0;
	v1 += message_holder.linear.x;
	v2 += message_holder.linear.x;

	roboteq_msgs::Command cmd1, cmd2;
	cmd1.mode = 0;
	cmd2.mode = 0;
	v1*= MAGIC_NUM;
	v2*= MAGIC_NUM;
	cmd1.setpoint = v1;
	cmd2.setpoint = v2;

	ch1.publish(cmd1);
	ch2.publish(cmd2);

}
int main(int argc, char **argv) 
{ 

	ros::init(argc,argv,"convert_twist");
	ros::NodeHandle nh_;
	ros::Subscriber my_subscriber_object= nh_.subscribe("cmd_vel",1,myCallback);
	ch1 = nh_.advertise<roboteq_msgs::Command>("ch1/cmd", 1);
	ch2 = nh_.advertise<roboteq_msgs::Command>("ch2/cmd", 1);
	ros::spin();
	return 0;
}