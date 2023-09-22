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

#include <cmath>

namespace rosneuro {

class smr_weel {

	public:
		smr_weel(void);
		~smr_weel(void);

		bool configure(
			std::vector<float> threshold_soft,
			std::vector<float> threshold_hard,
			std::vector<float> threshold_final
	    );

		void run(void);

		virtual void update();

		float input2angle(float input);


	protected:
		virtual void setup_scene(void);
		
		void on_received_neuro_data(const rosneuro_msgs::NeuroOutput& msg);

		void on_keyboard_event(const neurodraw::KeyboardEvent& event);

		neurodraw::Engine* 		engine_;
		ros::NodeHandle 	nh_;

		bool user_quit_;

		float input_min_;
		float input_max_;
		float angle_min_;
		float angle_max_;

		struct thresholds_angle {
			neurodraw::Rectangle* 	lline_;
			neurodraw::Rectangle* 	rline_;

            float left;
            float rigth;
            thresholds_angle(float l = 180.0f, float r = 0.0f, float circ = 0.725f, float len = 0.15f) : left(l), rigth(r) {
				this->lline_ = new neurodraw::Rectangle(0.01f, len, true, neurodraw::Palette::darkgray);
				this->rline_ = new neurodraw::Rectangle(0.01f, len, true, neurodraw::Palette::darkgray);

				this->rline_->move(0.0f, circ);
				this->lline_->move(0.0f, circ);
			
				this->rline_->rotate(rigth, 0.0f, 0.0f);
				this->lline_->rotate(left,  0.0f, 0.0f);
			}
        } probablility, command_a, command_b, cmd_f;


	private:
		ros::NodeHandle	 p_nh_;
		
		ros::Subscriber 	sub_neuro_;

		//neurodraw::Engine* 		engine_;
		
		neurodraw::Ring* 		ring_;
		
		neurodraw::Arc* 		arc_;
		neurodraw::Rectangle* 	mline_;
			
	
		float probability_angle = 0.0f;


};


}

#endif
