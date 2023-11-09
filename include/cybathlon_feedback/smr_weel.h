#ifndef CYBATHLON_FEEDBACK_WEEL_H_
#define CYBATHLON_FEEDBACK_WEEL_H_

#include <ros/ros.h>
#include <rosneuro_msgs/NeuroOutput.h>
#include <geometry_msgs/Twist.h>

#include "neurodraw/Ring.h"
#include "neurodraw/Arc.h"
#include "neurodraw/Cross.h"
#include "neurodraw/Line.h"
#include "neurodraw/Rectangle.h"
#include "neurodraw/Engine.h"
#include "neurodraw/Palette.h"
#include "neurodraw/EventKey.h"

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/Reconfigure.h>
//#include "cybathlon_feedback/FeedbackConfig.h"
#include "rosneuro_cybathlon_controller/AllControllerConfig.h"
#include "rosneuro_integrator_exponential/ExponentialMarginConfig.h"

#include <cmath>

namespace rosneuro {

using cybathlon_feedback   = rosneuro_cybathlon_controller::AllControllerConfig;
using dyncfg_feedback      = dynamic_reconfigure::Server<cybathlon_feedback>;

using exponential_feedback = rosneuro_integrator_exponential::ExponentialMarginConfig;
using dyncfg_exponential   = dynamic_reconfigure::Server<exponential_feedback>;

class smr_weel {

	public:
		smr_weel(void);
		~smr_weel(void);

		bool configure(
			std::vector<double> threshold_soft,
			std::vector<double> threshold_hard,
			std::vector<double> threshold_final
	    );
    
    bool configure();

		void run(void);

		virtual void update();

		float input2angle(float input);


	protected:
		virtual void setup_scene(void);
		
		void on_received_neuro_data(const rosneuro_msgs::NeuroOutput& msg);

		void on_keyboard_event(const neurodraw::KeyboardEvent& event);

		void on_request_reconfigure(cybathlon_feedback &config, uint32_t level);  
    void on_request_reconfigure_f(exponential_feedback &config, uint32_t level); 

		neurodraw::Engine* 		engine_;
		ros::NodeHandle 	nh_;

		bool user_quit_;

		double input_min_;
		double input_max_;
		double angle_min_;
		double angle_max_;

		struct thresholds_angle {
			neurodraw::Rectangle* 	lline_;
			neurodraw::Rectangle* 	rline_;

      double left;
      double rigth;
      
      thresholds_angle(float l = 180.0f, float r = 0.0f, neurodraw::Color color = neurodraw::Palette::darkgray , float circ = 0.725f, float len = 0.15f) : left(l), rigth(r) {
				this->lline_ = new neurodraw::Rectangle(0.01f, len, true, color);
				this->rline_ = new neurodraw::Rectangle(0.01f, len, true, color);

				this->rline_->move(0.0f, circ);
				this->lline_->move(0.0f, circ);
			
				this->rline_->rotate(rigth, 0.0f, 0.0f);
				this->lline_->rotate(left,  0.0f, 0.0f);
			}
    } probablility, command_a, command_b, cmd_f;

    neurodraw::Rectangle* middle_line_;

    std::vector<double> string2vector_converter(std::string msg);


	private:
		ros::NodeHandle	 p_nh_;
		
		ros::Subscriber 	sub_neuro_;

		//neurodraw::Engine* 		engine_;
		
		neurodraw::Ring* 		ring_;
		
		neurodraw::Arc* 		arc_;
		neurodraw::Rectangle* 	mline_;
			
	
		double probability_angle = 0.0f;

		dyncfg_feedback               recfg_srv_;
		dyncfg_feedback::CallbackType recfg_callback_type_;

    dyncfg_exponential::CallbackType recfg_exponential_callback_type_; 

		ros::ServiceClient client;
		ros::ServiceClient integrator_client;

		dynamic_reconfigure::Reconfigure srv;

    std::vector<double> thresholds_soft_ = {0.6,0.6}, thresholds_hard_ = {0.9,0.9}, thresholds_final_ = {1.0,1.0};
    std::string string_thresholds_soft_, string_thresholds_hard_, string_thresholds_final_, string_thresholds_initial_;


};

}

#endif
