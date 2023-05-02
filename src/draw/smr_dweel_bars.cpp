#include "cybathlon_feedback/smr_dweel_bars.h"

using namespace rosneuro;

smr_dweel_bars::smr_dweel_bars(void) : smr_dweel() {
		this->sub_bar_ = this->nh_.subscribe("/bar_status", 1, &smr_dweel_bars::on_received_bar_data, this);
}

void smr_dweel_bars::setup_scene(void){
		smr_dweel::setup_scene();
		this->abar_ = new neurodraw::Rectangle(0.15f, 1.1f, true, neurodraw::Palette::grey);
		this->bbar_ = new neurodraw::Rectangle(0.15f, 1.1f, true, neurodraw::Palette::grey);

		this->aline_ = new neurodraw::Rectangle(0.15f, 0.1f, true, neurodraw::Palette::white);
		this->bline_ = new neurodraw::Rectangle(0.15f, 0.1f, true, neurodraw::Palette::white);

		this->abar_->move(-1.0f, 0.0f);
		this->bbar_->move(1.0f,  0.0f);

		this->aline_->move(-1.0f, -0.5f);
		this->bline_->move( 1.0f, -0.5f);
		
		this->engine_->add(this->abar_);
		this->engine_->add(this->bbar_);
		this->engine_->add(this->aline_);
		this->engine_->add(this->bline_);

}

void smr_dweel_bars::on_received_bar_data(const std_msgs::Float32MultiArray& msg){
	bar1_ = msg.data.at(0);
	bar2_ = msg.data.at(1);
}

void smr_dweel_bars::update() {
	smr_dweel::update();

	this->aline_->move(-1.0f, bar1_-0.5f);
	this->bline_->move( 1.0f, bar2_-0.5f);
}

bool smr_dweel_bars::configure(
			std::vector<float> threshold_soft,
			std::vector<float> threshold_hard
	) {
	this->input_min_ = 0.0f;
	this->input_max_ = 1.0f;

	this->angle_min_ = 0.0f;
	this->angle_max_ = 180.0f;
	
	this->probablility = thresholds_angle(input2angle(threshold_soft[0]) , input2angle(1 - threshold_soft[1])  );
	this->command_a = thresholds_angle(input2angle(threshold_soft[0]) , input2angle(1 - threshold_soft[1])  );
	this->command_b = thresholds_angle(input2angle(threshold_hard[0]) , input2angle(1 - threshold_hard[1])  );

	this->setup_scene();

	return true;
}

