#include <ros/ros.h>
//#include "cybathlon_feedback/smr_dweel.h"
//#include "cybathlon_feedback/smr_dweel_bars.h"
#include "cybathlon_feedback/smr_weel.h"
#include "cybathlon_feedback/smr_weel_bars.h"


std::vector<double> f_converter(std::string msg){
	// If possible, always prefer std::vector to naked array
  std::vector<double> v;

	msg.replace(msg.find(", "), 2, " ");

  // Build an istream that holds the input string
  std::istringstream iss(msg);

  // Iterate over the istream, using >> to grab floats
  // and push_back to store them in the vector
  std::copy(std::istream_iterator<double>(iss),
        std::istream_iterator<double>(),
        std::back_inserter(v));

  // Put the result on standard out
  std::copy(v.begin(), v.end(),
        std::ostream_iterator<double>(std::cout, ", "));
  std::cout << "\n";
  return v;
	
}

std::vector<double> set_data(std::string topic_name, std::string default_string) {
	std::string parameters; 

	ros::param::param(topic_name, parameters, default_string);

	return f_converter(parameters);
}


int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "smr_weel");

	std::string def_th_s = "0.65, 0.65";
	std::string def_th_h = "0.9, 0.9";
	std::string def_th_f = "1, 1";

	
	std::vector<double> th_s = set_data("~threshold_soft", def_th_s);
	std::vector<double> th_h = set_data("~threshold_hard", def_th_h);
	std::vector<double> th_f = set_data("~threshold_final", def_th_f);
	
	rosneuro::smr_weel_bars wheel;

	if(wheel.configure(th_s,th_h, th_f) == false) {
		ROS_ERROR("Wheel configuration failed");
		ros::shutdown();
		return 1;
	}

	wheel.run();

	ros::shutdown();
	

	return 0;

}
