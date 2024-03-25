#include <ros/ros.h>
#include "cybathlon_feedback/DoubleThresholdWheel.h"

int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "doublethresholdwheel");

	ros::NodeHandle nh;
	cybathlon::DoubleThresholdWheel wheel;

	if(wheel.configure() == false) {
		ROS_ERROR("DoubleThresholdWheel configuration failed");
		ros::shutdown();
		return 0;
	}


	wheel.run();

	ros::shutdown();

	return 0;
}
