#include "cybathlon_feedback/smr_weel.h"

namespace rosneuro {


smr_weel::smr_weel(void) : p_nh_("~") {

	this->sub_neuro_ = this->nh_.subscribe("/integrator/neuroprediction", 1, &smr_weel::on_received_neuro_data, this);

	this->engine_ = new neurodraw::Engine("smr_dweel");
	this->engine_->on_keyboard(&smr_weel::on_keyboard_event, this);

	this->user_quit_ = false;
}

smr_weel::~smr_weel(void) {

	if(this->engine_ != nullptr)
		delete this->engine_;

}

bool smr_weel::configure(
			std::vector<float> threshold_soft,
			std::vector<float> threshold_hard,
			std::vector<float> threshold_final
	) {
	this->input_min_ = 0.0f;
	this->input_max_ = 1.0f;

	this->angle_min_ = 0.0f;
	this->angle_max_ = 180.0f;
	
	this->probablility = thresholds_angle(input2angle(threshold_soft[0]) , input2angle(1 - threshold_soft[1])  );
	this->command_a = thresholds_angle(input2angle(threshold_soft[0]) , input2angle(1 - threshold_soft[1])  );
	this->command_b = thresholds_angle(input2angle(threshold_hard[0]) , input2angle(1 - threshold_hard[1])  );
	this->cmd_f = thresholds_angle(input2angle(threshold_final[0]) , input2angle(1 - threshold_final[1])  );

	this->setup_scene();

	return true;
}


void smr_weel::setup_scene(void) {
	
	this->ring_  = new neurodraw::Ring(0.8f, 0.15f, neurodraw::Palette::grey);

	this->arc_   = new neurodraw::Arc(0.8f, 0.15f, 2.0 * M_PI / 3.0f, neurodraw::Palette::lightgrey);
	this->mline_ = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::green);
	
	this->arc_->rotate(30);
	this->mline_->move(0.0f, 0.725f);

	this->engine_->add(this->ring_);

	this->engine_->add(this->arc_);
	this->engine_->add(this->probablility.lline_);
	this->engine_->add(this->probablility.rline_);
	
	this->engine_->add(this->command_a.rline_);
	this->engine_->add(this->command_a.lline_);
	this->engine_->add(this->command_b.rline_);
	this->engine_->add(this->command_b.lline_);

	this->engine_->add(this->cmd_f.rline_);
	this->engine_->add(this->cmd_f.lline_);
	
	this->engine_->add(this->mline_);

}

void smr_weel::update() {
	this->arc_->rotate(probability_angle - 60.0f);
	
	this->mline_->rotate(probability_angle, 0.0f, 0.0f);
}

void smr_weel::run(void) {
	ros::Rate r(100);
	
	while(ros::ok() & this->user_quit_ == false) {
		ros::spinOnce();
		this->update();
		r.sleep();
	}

}

void smr_weel::on_received_neuro_data(const rosneuro_msgs::NeuroOutput& msg) {
	float input;
	input = msg.softpredict.data.at(0);
	probability_angle = this->input2angle(input);
}

void smr_weel::on_keyboard_event(const neurodraw::KeyboardEvent& event) {

	if(event.state == 0)
		return;

	switch(event.sym) {
	    case neurodraw::EventKey::ESCAPE:
	   	 this->engine_->quit();
		 this->user_quit_ = true;
	   	 break;
	}
}

float smr_weel::input2angle(float input) {

	float b, a, xmax, xmin, angle;

	b    = this->angle_max_;
	a    = this->angle_min_;
	xmax = this->input_max_;
	xmin = this->input_min_;
	
	angle = (b-a) * ( (input - xmin) / (xmax - xmin) ) + a;

	return angle;

}

}
