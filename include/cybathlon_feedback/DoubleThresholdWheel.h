#ifndef CYBATHLON_FEEDBACK_WHEEL_
#define CYBATHLON_FEEDBACK_WHEEL_

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>

#include <rosneuro_msgs/NeuroOutput.h>
#include <rosneuro_msgs/NeuroEvent.h>
#include <rosneuro_feedback_wheel/SingleWheel.h>

#include <cybathlon_commons/commons.h>
#include "cybathlon_feedback/CybathlonWheelConfig.h"

namespace cybathlon {

using cybathlon_config_wheel = cybathlon_feedback::CybathlonWheelConfig;
using dyncfg_cybathlon_wheel = dynamic_reconfigure::Server<cybathlon_config_wheel>;
using InputState = cybathlon::BCICommand;

class DoubleThresholdWheel : public rosneuro::feedback::SingleWheel {

	public:
		DoubleThresholdWheel(void);
		virtual ~DoubleThresholdWheel(void);


		bool configure(void);
		void run(void);

		void on_request_reconfigure(cybathlon_config_wheel &config, uint32_t level);

	private:
		void on_receive_game_event(const rosneuro_msgs::NeuroEvent& msg);	
		void on_receive_artifact_event(const rosneuro_msgs::NeuroEvent& msg);	
		void on_receive_neuroprediction(const rosneuro_msgs::NeuroOutput& msg);
		
		InputState on_state_transition(float input, cybathlon::InputState cstate);
		void setup_wheel(void);
		void set_soft_threshold_right(double threshold);
		void set_hard_threshold_right(double threshold);
		void set_soft_threshold_left(double threshold);
		void set_hard_threshold_left(double threshold);

		void show_artifact(void);
		void hide_artifact(void);

		cybathlon_config_wheel get_wheel_parameters(const cybathlon::GameTask& task);
		cybathlon_config_wheel* get_task_config(const cybathlon::GameTask& task);
		void set_wheel_parameters(const cybathlon_config_wheel& config);

	private:
		ros::NodeHandle    nh_;
		ros::NodeHandle    p_nh_;
		ros::Subscriber    subctr_;
		ros::Subscriber    subgam_;
		ros::Subscriber    subart_;
		ros::Publisher     pubevt_;
		ros::ServiceClient srv_reset_;

		std::vector<int> classes_;
		bool has_new_input_;
		float ctr_input_;
		InputState cstate_;
		InputState nstate_;
		bool has_reset_on_soft_;
		bool has_reset_on_hard_;

		neurodraw::Rectangle* 	soft_left_;
		neurodraw::Rectangle* 	soft_right_;
		neurodraw::Rectangle* 	hard_left_;
		neurodraw::Rectangle* 	hard_right_;

		double soft_threshold_left_;
		double soft_threshold_right_;
		double hard_threshold_left_;
		double hard_threshold_right_;
		
		cybathlon_config_wheel  config_wheelchair_;
		cybathlon_config_wheel  config_roboticarm_;
		cybathlon_config_wheel  config_screencursor_;
		cybathlon_config_wheel* config_;
		
		dyncfg_cybathlon_wheel* recfg_srv_;
  		dyncfg_cybathlon_wheel::CallbackType recfg_callback_type_;
		boost::recursive_mutex recfg_mutex_;

};




}



#endif
