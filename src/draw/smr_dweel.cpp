#include "cybathlon_feedback/smr_dweel.h"

namespace rosneuro {


smr_dweel::smr_dweel(void) : p_nh_("~") {

	this->sub_neuro_ = this->nh_.subscribe("/integrator/neuroprediction", 1, &smr_dweel::on_received_neuro_data, this);
	this->sub_yaw_   = this->nh_.subscribe("/cmd_vel", 1, &smr_dweel::on_received_yaw_data, this);

	this->engine_ = new neurodraw::Engine("smr_dweel");
	this->engine_->on_keyboard(&smr_dweel::on_keyboard_event, this);

	this->user_quit_ = false;

}

smr_dweel::~smr_dweel(void) {

	if(this->engine_ != nullptr)
		delete this->engine_;

}

bool smr_dweel::configure(
			std::vector<float> threshold,
			std::vector<float> range_a,
			std::vector<float> range_b
	) {
	this->input_min_ = 0.0f;
	this->input_max_ = 1.0f;

	this->angle_min_ = 0.0f;
	this->angle_max_ = 180.0f;
	
	this->probablility = thresholds_angle(input2angle(threshold[0]) , input2angle(threshold[1])  );
	this->command_a = thresholds_angle(range_a[0] * 180 / M_PI + 90 ,range_a[1] * 180 / M_PI + 90 , 0.525f);
	this->command_b = thresholds_angle(range_b[0] * 180 / M_PI + 90 ,range_b[1] * 180 / M_PI + 90 , 0.525f);

	this->setup_scene();

	return true;
}

void smr_dweel::setup_scene(void) {
	
	this->external_ring_  = new neurodraw::Ring(0.8f, 0.15f, neurodraw::Palette::grey);
	this->internal_ring_ = new neurodraw::Ring(0.6f, 0.15f, neurodraw::Palette::lightgrey);

	this->arc_   = new neurodraw::Arc(0.8f, 0.15f, 2.0 * M_PI / 3.0f, neurodraw::Palette::lightgrey);
	this->mline_ = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::green);
	
	this->yawline_ = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::black);
	this->yawline_->move(0.0f, 0.525f);

	this->arc_->rotate(30);
	this->mline_->move(0.0f, 0.725f);

	this->engine_->add(this->external_ring_);
	this->engine_->add(this->internal_ring_);

	this->engine_->add(this->yawline_);

	this->engine_->add(this->arc_);
	this->engine_->add(this->probablility.lline_);
	this->engine_->add(this->probablility.rline_);
	
	this->engine_->add(this->command_a.rline_);
	this->engine_->add(this->command_a.lline_);
	this->engine_->add(this->command_b.rline_);
	this->engine_->add(this->command_b.lline_);
	
	this->engine_->add(this->mline_);

}

void smr_dweel::update() {
	this->arc_->rotate(probability_angle - 60.0f);
	
	this->mline_->rotate(probability_angle, 0.0f, 0.0f);
	
	this->yawline_->rotate( 90 -  yaw * 180 / M_PI  , 0.0f, 0.0f);
}

void smr_dweel::run(void) {
	ros::Rate r(100);
	
	while(ros::ok() & this->user_quit_ == false) {

		ros::spinOnce();
		this->update();
		r.sleep();
	}

}

void smr_dweel::on_received_neuro_data(const rosneuro_msgs::NeuroOutput& msg) {
	float input;
	input = msg.softpredict.data.at(0);
	probability_angle = this->input2angle(input);
}

void smr_dweel::on_received_yaw_data(const geometry_msgs::Twist& msg) {
	yaw = msg.angular.z;
}

void smr_dweel::on_keyboard_event(const neurodraw::KeyboardEvent& event) {

	if(event.state == 0)
		return;

	switch(event.sym) {
	    case neurodraw::EventKey::ESCAPE:
	   	 this->engine_->quit();
		 this->user_quit_ = true;
	   	 break;
	}
}

float smr_dweel::input2angle(float input) {

	float b, a, xmax, xmin, angle;

	b    = this->angle_max_;
	a    = this->angle_min_;
	xmax = this->input_max_;
	xmin = this->input_min_;
	
	angle = (b-a) * ( (input - xmin) / (xmax - xmin) ) + a;

	return angle;

}

}
