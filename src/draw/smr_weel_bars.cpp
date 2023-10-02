#include "cybathlon_feedback/smr_weel_bars.h"

using namespace rosneuro;

smr_weel_bars::smr_weel_bars(void) : smr_weel() {
		this->sub_bar_ = this->nh_.subscribe("/bar_status", 1, &smr_weel_bars::on_received_bar_data, this);
}

void smr_weel_bars::setup_scene(void){
		smr_weel::setup_scene();
		this->abar_ = new neurodraw::Rectangle(0.15f, 1.1f, true, neurodraw::Palette::grey);
		this->bbar_ = new neurodraw::Rectangle(0.15f, 1.1f, true, neurodraw::Palette::grey);

		this->aline_ = new neurodraw::Rectangle(0.15f, 0.1f, true, neurodraw::Palette::white);
		this->bline_ = new neurodraw::Rectangle(0.15f, 0.1f, true, neurodraw::Palette::white);

		this->a_ = new neurodraw::RegularShape(0.15f, 3, false,  neurodraw::Palette::white);
		this->b_ = new neurodraw::RegularShape(0.15f, 4, false,  neurodraw::Palette::white);

		this->a_->move(-1.0f,  0.75f);
		this->b_->move( 1.0f,  0.8f);

		this->a_->rotate( 90);
		this->b_->rotate(-45);

		this->abar_->move(-1.0f, 0.0f);
		this->bbar_->move(1.0f,  0.0f);

		this->aline_->move(-1.0f, -0.5f);
		this->bline_->move( 1.0f, -0.5f);
		
		this->engine_->add(this->abar_);
		this->engine_->add(this->bbar_);
		this->engine_->add(this->aline_);
		this->engine_->add(this->bline_);

		this->engine_->add(this->a_);
		this->engine_->add(this->b_);
}

void smr_weel_bars::on_received_bar_data(const std_msgs::Float32MultiArray& msg){
	bar1_ = msg.data.at(0);
	bar2_ = msg.data.at(1);
}

void smr_weel_bars::update() {
	smr_weel::update();

	this->aline_->move(-1.0f, bar1_-0.5f);
	this->bline_->move( 1.0f, bar2_-0.5f);
}
