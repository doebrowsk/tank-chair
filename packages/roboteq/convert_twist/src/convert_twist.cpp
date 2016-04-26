#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <roboteq_msgs/Command.h>
#include <roboteq_msgs/Feedback.h>
#include <sensor_msgs/JointState.h>

//this node takes a geometry msgs twist in m/s and rad/s
//and publishes roboteq_msgs to the 2 channels
ros::Publisher ch1, ch2;

float MAX_COMMAND = 721.0;
float MAX_SPEED = 1.128;
float BASE_RADIUS = 0.3505;
float RADS_PER_TICK = 0.0426;
float LEFT_OVER_RIGHT = 0.997;
float g_pos1, g_pos2, g_vel1, g_vel2;

float LOOP_RATE = 50.0;
float MAX_LIN_ACEL = 1.0;
float MAX_ANG_ACEL = 1.0;

int DRIVING_DIRECTION = -1;



geometry_msgs::Twist g_last_sent_cmd;
geometry_msgs::Twist g_last_received_cmd;


void twistCallback(const geometry_msgs::Twist& message_holder) 
{	
	g_last_received_cmd = message_holder;

	// geometry_msgs::Twist cmd_to_motor = message_holder;
	// float dv_lin = cmd_to_motor.linear.x-g_last_cmd.linear.x;
	// float dv_ang = cmd_to_motor.angular.z-g_last_cmd.angular.z;

	// if(dv_lin>MAX_LIN_ACEL/LOOP_RATE){
	// 	cmd_to_motor.linear.x = g_last_cmd.linear.x + MAX_LIN_ACEL/LOOP_RATE;
	// 	ROS_INFO("limiting from %f to %f", message_holder.linear.x, cmd_to_motor.linear.x);
	// }else if(dv_lin*-1.0>MAX_LIN_ACEL/LOOP_RATE){
	// 	cmd_to_motor.linear.x = g_last_cmd.linear.x - MAX_LIN_ACEL/LOOP_RATE;
	// }

	// if(dv_ang > MAX_ANG_ACEL/LOOP_RATE){
	// 	cmd_to_motor.angular.z = g_last_cmd.angular.z + MAX_ANG_ACEL/LOOP_RATE;
	// }else if(dv_ang*-1.0>MAX_ANG_ACEL/LOOP_RATE){
	// 	cmd_to_motor.angular.z = g_last_cmd.angular.z - MAX_ANG_ACEL/LOOP_RATE;
	// }

	// g_last_cmd = cmd_to_motor;

	// float v1,v2;
	
	// v1 = cmd_to_motor.linear.x;
	// v2 = cmd_to_motor.linear.x;

	// v1*=MAX_COMMAND/MAX_SPEED;
	// v2*=MAX_COMMAND/MAX_SPEED;

	// v2 += (cmd_to_motor.angular.z)*MAX_COMMAND/MAX_SPEED*BASE_RADIUS;
	// v1 -= (cmd_to_motor.angular.z)*MAX_COMMAND/MAX_SPEED*BASE_RADIUS;

	// v1/= LEFT_OVER_RIGHT;
	// v2*= LEFT_OVER_RIGHT;

	// v1 = std::max(-1.0f*MAX_COMMAND,std::min(v1,MAX_COMMAND));
	// v2 = std::max(-1.0f*MAX_COMMAND,std::min(v2,MAX_COMMAND));

	// roboteq_msgs::Command cmd1, cmd2;
	// cmd1.mode = 0;
	// cmd2.mode = 0;
	// cmd1.setpoint = v1;
	// cmd2.setpoint = v2;

	// ch1.publish(cmd1);
	// ch2.publish(cmd2);

}
void feedback1(const roboteq_msgs::Feedback& message_holder){

	if(DRIVING_DIRECTION==-1){
		g_pos2 = message_holder.measured_position*RADS_PER_TICK*DRIVING_DIRECTION;
		g_vel2 = message_holder.measured_velocity*RADS_PER_TICK*DRIVING_DIRECTION;
	}

}
void feedback2(const roboteq_msgs::Feedback& message_holder){
	
	if(DRIVING_DIRECTION==-1){
		g_pos1 = message_holder.measured_position*RADS_PER_TICK*DRIVING_DIRECTION;
		g_vel1 = message_holder.measured_velocity*RADS_PER_TICK*DRIVING_DIRECTION;
	}
}
int main(int argc, char **argv) 
{ 


	ros::init(argc,argv,"convert_twist");
	ros::NodeHandle nh_;
	ros::Subscriber cmd_converter= nh_.subscribe("cmd_vel",1,twistCallback);
	ros::Subscriber fdbk1= nh_.subscribe("ch1/feedback",1,feedback1);
	ros::Subscriber fdbk2= nh_.subscribe("ch2/feedback",1,feedback2);
	ch1 = nh_.advertise<roboteq_msgs::Command>("ch1/cmd", 1);
	ch2 = nh_.advertise<roboteq_msgs::Command>("ch2/cmd", 1);
	ros::Publisher joints = nh_.advertise<sensor_msgs::JointState>("joint_states",1);
	g_pos1 = 0;
	g_pos2 = 0;
	g_vel1 = 0;
	g_vel2 = 0;

	g_last_sent_cmd.linear.x = 0;
	g_last_sent_cmd.angular.z = 0;

	ros::Rate looprate(LOOP_RATE);
	while (ros::ok()){
		sensor_msgs::JointState state;
		state.name.push_back("ch1");
		state.position.push_back(g_pos1);
		state.velocity.push_back(g_vel1);
		state.name.push_back("ch2");
		state.position.push_back(g_pos2);
		state.velocity.push_back(g_vel2);
		state.header.stamp = ros::Time::now();
		joints.publish(state);
		looprate.sleep();
		ros::spinOnce();



		geometry_msgs::Twist cmd_to_motor = g_last_received_cmd;
		float dv_lin = cmd_to_motor.linear.x-g_last_sent_cmd.linear.x;
		float dv_ang = cmd_to_motor.angular.z-g_last_sent_cmd.angular.z;

		if(dv_lin>MAX_LIN_ACEL/LOOP_RATE){
			cmd_to_motor.linear.x = g_last_sent_cmd.linear.x + MAX_LIN_ACEL/LOOP_RATE;
			//ROS_INFO("limiting from %f to %f", g_last_received_cmd.linear.x, cmd_to_motor.linear.x);
		}else if(dv_lin*-1.0>MAX_LIN_ACEL/LOOP_RATE){
			cmd_to_motor.linear.x = g_last_sent_cmd.linear.x - MAX_LIN_ACEL/LOOP_RATE;
		}

		if(dv_ang > MAX_ANG_ACEL/LOOP_RATE){
			cmd_to_motor.angular.z = g_last_sent_cmd.angular.z + MAX_ANG_ACEL/LOOP_RATE;
		}else if(dv_ang*-1.0>MAX_ANG_ACEL/LOOP_RATE){
			cmd_to_motor.angular.z = g_last_sent_cmd.angular.z - MAX_ANG_ACEL/LOOP_RATE;
		}

		g_last_sent_cmd = cmd_to_motor;

		float v1,v2;
		
		v1 = cmd_to_motor.linear.x;
		v2 = cmd_to_motor.linear.x;

		v1*=MAX_COMMAND/MAX_SPEED*DRIVING_DIRECTION;
		v2*=MAX_COMMAND/MAX_SPEED*DRIVING_DIRECTION;

		v2 += (cmd_to_motor.angular.z)*MAX_COMMAND/MAX_SPEED*BASE_RADIUS;
		v1 -= (cmd_to_motor.angular.z)*MAX_COMMAND/MAX_SPEED*BASE_RADIUS;

		v1/= LEFT_OVER_RIGHT;
		v2*= LEFT_OVER_RIGHT;

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
	return 0;
}
