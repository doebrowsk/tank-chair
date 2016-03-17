#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <roboteq_msgs/Command.h>

//this node takes a geometry msgs twist in m/s and rad/s
//and publishes roboteq_msgs to the 2 channels
ros::Publisher ch1, ch2;

float MAX_COMMAND = 205.0;
float MAX_SPEED = 1.22;
float BASE_RADIUS = 0.3;

void myCallback(const geometry_msgs::Twist& message_holder) 
{
	float v1,v2;
	
	v1 += message_holder.linear.x;
	v2 += message_holder.linear.x;

	v1*=MAX_COMMAND/MAX_SPEED;
	v2*=MAX_COMMAND/MAX_SPEED;

	v2 += (message_holder.angular.z/2.0)*MAX_COMMAND/MAX_SPEED*BASE_RADIUS;
	v1 -= (message_holder.angular.z/2.0)*MAX_COMMAND/MAX_SPEED*BASE_RADIUS;
	v1 = std::max(-1.0f*MAX_COMMAND,std::min(v1,MAX_COMMAND));
	v2 = std::max(-1.0f*MAX_COMMAND,std::min(v2,MAX_COMMAND));

	roboteq_msgs::Command cmd1, cmd2;
	cmd1.mode = 0;
	cmd2.mode = 0;
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
