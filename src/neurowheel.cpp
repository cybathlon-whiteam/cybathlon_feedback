#include <ros/ros.h>
//#include "cybathlon_feedback/smr_dweel.h"
//#include "cybathlon_feedback/smr_dweel_bars.h"
#include "cybathlon_feedback/smr_weel.h"
#include "cybathlon_feedback/smr_weel_bars.h"

int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "smr_weel");

	rosneuro::smr_weel_bars wheel;

	if(wheel.configure() == false) {
		ROS_ERROR("Wheel configuration failed");
		ros::shutdown();
		return 1;
	}

	wheel.run();

	ros::shutdown();
	

	return 0;

}
