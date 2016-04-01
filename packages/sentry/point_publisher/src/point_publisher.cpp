#include <pub_des_state/pub_des_state.h>
#include <iostream>
using namespace std;

int main(int argc, char **argv) {
	ros::init(argc, argv, "point_publisher");
	ros::NodeHandle nh;
	//instantiate a desired-state publisher object
	DesStatePublisher desStatePublisher(nh);
	ros::Rate looprate(1 / dt); //timer for fixed publication rate
	desStatePublisher.set_init_pose(0,0,0);
	desStatePublisher.append_path_queue(1.0,0.0,0);
	desStatePublisher.append_path_queue(1.0,6.0,0);
	desStatePublisher.append_path_queue(-6.0,6.0,0);
	desStatePublisher.append_path_queue(-6.0,0.0,0);
	desStatePublisher.append_path_queue(-8.0,6.0,0);
	desStatePublisher.append_path_queue(2.0,6.0,0);
	desStatePublisher.append_path_queue(0.0,0.0,0);



	while (ros::ok()) {

		desStatePublisher.pub_next_state();

		if(desStatePublisher.get_estop_trigger())
		{
			ROS_INFO("estop is pressed");
		}

		ros::spinOnce();
		looprate.sleep();
	}

}
