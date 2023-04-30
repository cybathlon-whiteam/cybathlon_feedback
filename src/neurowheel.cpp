#include <ros/ros.h>
#include "cybathlon_feedback/smr_dweel.h"

std::vector<float> f_converter(std::string msg){
	// If possible, always prefer std::vector to naked array
  std::vector<float> v;

	msg.replace(msg.find(", "), 2, " ");

  // Build an istream that holds the input string
  std::istringstream iss(msg);

  // Iterate over the istream, using >> to grab floats
  // and push_back to store them in the vector
  std::copy(std::istream_iterator<float>(iss),
        std::istream_iterator<float>(),
        std::back_inserter(v));

  // Put the result on standard out
  std::copy(v.begin(), v.end(),
        std::ostream_iterator<float>(std::cout, ", "));
  std::cout << "\n";
  return v;
	
}

std::vector<float> set_data(std::string topic_name, std::string default_string) {
	std::string parameters; 
	std::vector<float> v_parameters;

	ros::param::param(topic_name, parameters, default_string);

	return f_converter(parameters);
}


int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "smr_dweel");

	std::string def_thr = "0.2, 0.9";
	std::string def_a   = "-2.0, -2.9";
	std::string def_b   = "2.0, 2.9";

	std::vector<float> threshold = set_data("~threshold", def_thr);
	std::vector<float> range_a   = set_data("~range_a", def_a);
	std::vector<float> range_b   = set_data("~range_b", def_b);

	rosneuro::smr_dweel wheel;

	if(wheel.configure(threshold,range_a,range_b) == false) {
		ROS_ERROR("Wheel configuration failed");
		ros::shutdown();
		return 1;
	}

	wheel.run();

	ros::shutdown();

	return 0;

}
