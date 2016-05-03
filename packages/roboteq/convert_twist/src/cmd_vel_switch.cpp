#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

ros::Publisher out;
std_msgs::Int32 mode;
geometry_msgs::Twist joy;
geometry_msgs::Twist rviz;
geometry_msgs::Twist bug;
float LOOP_RATE = 100.0;

void modecb(const std_msgs::Int32& message_holder){
  mode = message_holder;
}

void joycmd(const geometry_msgs::Twist& message_holder){

	joy = message_holder;

}

void rvizcmd(const geometry_msgs::Twist& message_holder){

	rviz = message_holder;

}
void gpscmd(const geometry_msgs::Twist& message_holder){

  bug = message_holder;

}



int main(int argc, char **argv) {
    ros::init(argc, argv, "cmd_vel_switch");
    ros::NodeHandle nh; 
    ros::Publisher cmder = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber modesub= nh.subscribe("cmd_mode",1,modecb);
    ros::Subscriber joysub= nh.subscribe("joystick/cmd_vel",1,joycmd);
    ros::Subscriber rvizsub= nh.subscribe("rviz/cmd_vel",1,rvizcmd);
    ros::Subscriber gpssub= nh.subscribe("gps_bug/cmd_vel",1,gpscmd);

    
    mode.data = 0;
    
    
    ros::Rate looprate(LOOP_RATE);
    while (ros::ok()) 
    {
       if (mode.data==0){
       		cmder.publish(joy);
       }
       if (mode.data==1){
       		cmder.publish(rviz);
       }
       if (mode.data==2){
          cmder.publish(bug);
       }
      	looprate.sleep();
		ros::spinOnce();
    }
}