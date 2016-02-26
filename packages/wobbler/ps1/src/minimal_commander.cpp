#include<ros/ros.h> 
#include<std_msgs/Float64.h> 
#include <math.h>  



int main(int argc, char **argv) {
	double amplitude = 20.0;
	double frequency = 50.0;
	std_msgs::Float64 velocity;//velocity to publish
	double time = 0;//time ++
    ros::init(argc, argv, "minimal_commander"); //name this node minimal_commander
    ros::NodeHandle nh; // node handle 
    //not subscribing to anything

    //publishing vel_cmd 
    ros::Publisher my_publisher_object = nh.advertise<std_msgs::Float64>("vel_cmd", 1);

    double sample_rate = 10; // compute the corresponding update frequency 
    ros::Rate naptime(sample_rate); // use to regulate loop rate 
    while (ros::ok()) {
	velocity.data = amplitude * sin(frequency * time);//y = A sin(Fx)
	time = time + .01;//time goes up
	my_publisher_object.publish(velocity); // publish the control effort computed by this 
	
	ROS_INFO("velocity is = %f", velocity.data);
	ros::spinOnce(); //allow data update from callback
        naptime.sleep(); // wait for remainder of specified periodS; 
    }
    return 0; // should never get here, unless roscore dies 
} 
