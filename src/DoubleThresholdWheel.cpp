#include "cybathlon_feedback/DoubleThresholdWheel.h"


namespace cybathlon {

DoubleThresholdWheel::DoubleThresholdWheel(void) : rosneuro::feedback::SingleWheel("DoubleThresholdWheel"), p_nh_("~") {

	// Publisher and Subscriber
	this->subctr_ = this->nh_.subscribe("/smr/neuroprediction/integrated", 1, 
									 &DoubleThresholdWheel::on_receive_neuroprediction, this);
	this->subgam_ = this->nh_.subscribe("/events/bus", 1, 
									 &DoubleThresholdWheel::on_receive_game_event, this);
	this->subart_ = this->nh_.subscribe("/events/bus", 1, 
									 &DoubleThresholdWheel::on_receive_artifact_event, this);
	this->pubevt_ = this->nh_.advertise<rosneuro_msgs::NeuroEvent>("/events/bus", 1);

	this->srv_reset_ = this->nh_.serviceClient<std_srvs::Empty>("/integrator/reset");

	this->has_new_input_ = false;
	this->ctr_input_     = 0.5f;
	this->cstate_ = cybathlon::InputState::None;
	this->nstate_ = cybathlon::InputState::None;

	// Graphic setup
	this->setup_wheel();

	// Only for developing in virtual box
	this->engine_->set_fps_tolerance(40.0f);

	// By deafult starting with the wheelchair task
	this->config_ = this->get_task_config(cybathlon::GameTask::Wheelchair);

	// Bind dynamic reconfigure callback
	this->recfg_srv_ = new dyncfg_cybathlon_wheel(this->recfg_mutex_);
	this->recfg_callback_type_ = boost::bind(&DoubleThresholdWheel::on_request_reconfigure, this, _1, _2);
	this->recfg_srv_->setCallback(this->recfg_callback_type_);
	
}

DoubleThresholdWheel::~DoubleThresholdWheel(void) {
	delete this->recfg_srv_;
}


void DoubleThresholdWheel::setup_wheel(void) {

	// Configure SingleWheel hiding the default thresholds and the min/max lines
	this->lline_->hide();
	this->rline_->hide();

	this->minline_->hide();
	this->maxline_->hide();

	// Creating soft and hard thresholds
	this->soft_left_  = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::dimgray);
	this->soft_right_ = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::dimgray);
	this->hard_left_  = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::royalblue);
	this->hard_right_ = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::firebrick);
	
	this->soft_left_ ->move(0.0f, 0.725f);
	this->soft_right_->move(0.0f, 0.725f);
	this->hard_left_ ->move(0.0f, 0.725f);
	this->hard_right_->move(0.0f, 0.725f);

	this->soft_left_ ->rotate(0.0f, 0.0f, 0.0f);
	this->soft_right_->rotate(0.0f, 0.0f, 0.0f);
	this->hard_left_ ->rotate(0.0f, 0.0f, 0.0f);
	this->hard_right_->rotate(0.0f, 0.0f, 0.0f);

	this->engine_->add(this->soft_left_);
	this->engine_->add(this->soft_right_);
	this->engine_->add(this->hard_left_);
	this->engine_->add(this->hard_right_);

	// Artifact warning (using circle_ from the parent class)	
	this->circle_->set_color(neurodraw::Palette::orange);
	
}

bool DoubleThresholdWheel::configure(void) {

	// Getting classes
	if(this->p_nh_.getParam("classes", this->classes_) == false) {
		ROS_ERROR("[%s] Parameter 'classes' is mandatory", this->name().c_str());
		return false;
	}

	// Task configuration
	this->config_wheelchair_   = this->get_wheel_parameters(cybathlon::GameTask::Wheelchair);
	this->config_roboticarm_   = this->get_wheel_parameters(cybathlon::GameTask::RoboticArm);
	this->config_screencursor_ = this->get_wheel_parameters(cybathlon::GameTask::ScreenCursor);

	// By deafult starting with the wheelchair task
	this->config_ = this->get_task_config(cybathlon::GameTask::Wheelchair);

	// Update wheel parameters with the configuration 
	this->set_wheel_parameters(*(this->config_));

	// Update reconfigure with the new configuration
	this->recfg_srv_->updateConfig(*(this->config_));


	return true;
}


cybathlon_config_wheel DoubleThresholdWheel::get_wheel_parameters(const cybathlon::GameTask& task) {

	std::string stask;
	cybathlon_config_wheel config;

	// Get task name
	stask = cybathlon::to_string(task);

	// Get task parameters from server
	this->p_nh_.param<double>(stask + "/soft_threshold_left",  config.soft_threshold_left,  0.7f);
	this->p_nh_.param<double>(stask + "/soft_threshold_right", config.soft_threshold_right, 0.7f);
	this->p_nh_.param<double>(stask + "/hard_threshold_left",  config.hard_threshold_left,  0.9f);
	this->p_nh_.param<double>(stask + "/hard_threshold_right", config.hard_threshold_right, 0.9f);
	this->p_nh_.param<bool>(stask + "/reset_on_soft", config.reset_on_soft, false);
	this->p_nh_.param<bool>(stask + "/reset_on_hard", config.reset_on_hard, true);

	return config;
}

void DoubleThresholdWheel::set_wheel_parameters(const cybathlon_config_wheel& config) {

	this->set_soft_threshold_left(config.soft_threshold_left);
	this->set_soft_threshold_right(config.soft_threshold_right);
	this->set_hard_threshold_left(config.hard_threshold_left);
	this->set_hard_threshold_right(config.hard_threshold_right); 
	this->has_reset_on_soft_ = config.reset_on_soft;
	this->has_reset_on_hard_ = config.reset_on_hard;
}

cybathlon_config_wheel* DoubleThresholdWheel::get_task_config(const cybathlon::GameTask& task) {
	
	cybathlon_config_wheel* config;

	switch(task) {
		case cybathlon::GameTask::Wheelchair:
			config = &(this->config_wheelchair_);
			break;
		case cybathlon::GameTask::RoboticArm:
			config = &(this->config_roboticarm_);
			break;
		case cybathlon::GameTask::ScreenCursor:
			config = &(this->config_screencursor_);
			break;
		default:
			config = &(this->config_wheelchair_);
			break;
	}

	return config;
}

void DoubleThresholdWheel::run(void) {


	ros::Rate r(50.0f);

	while(ros::ok()) {

		this->nstate_ = this->on_state_transition(this->ctr_input_, this->cstate_);
		
		if(this->has_new_input_ = true) {
			
			this->move(this->input2angle(this->ctr_input_));
			this->has_new_input_ = false;
		}


		ros::spinOnce();
		r.sleep();
		
		this->cstate_ = this->nstate_;
	}

}

void DoubleThresholdWheel::show_artifact(void) {
	this->circle_->show();
	this->arc_->set_alpha(0.3f);
}

void DoubleThresholdWheel::hide_artifact(void) {
	this->circle_->hide();
	this->arc_->set_alpha(1.0f);
}

cybathlon::InputState DoubleThresholdWheel::on_state_transition(float input, cybathlon::InputState cstate) {

	rosneuro_msgs::NeuroEvent evtmsg;
	std_srvs::Empty resetmsg;

	cybathlon::InputState nstate, rstate;
	rstate = cstate;

	if(input >= this->hard_threshold_left_) {
		nstate = cybathlon::InputState::HardLeft;
	} else if(input <= this->hard_threshold_right_) {
		nstate = cybathlon::InputState::HardRight;
	} else if(input >= this->soft_threshold_left_) {
		nstate = cybathlon::InputState::SoftLeft;
	} else if(input <= this->soft_threshold_right_) {
		nstate = cybathlon::InputState::SoftRight;
	} else {
		nstate = cybathlon::InputState::None;
	}

	if(nstate != cstate) {
		ROS_INFO("[%s] State transition: %s->%s", this->name().c_str(), 
												  cybathlon::to_string(cstate).c_str(), 
												  cybathlon::to_string(nstate).c_str());

		// Publish the event
		evtmsg.header.stamp = ros::Time::now();
		evtmsg.event = static_cast<int>(nstate);
		this->pubevt_.publish(evtmsg);

		// Call for reset if requested
		if(this->has_reset_on_soft_ && (nstate == cybathlon::InputState::SoftLeft ||
										nstate == cybathlon::InputState::SoftRight)) {
			this->srv_reset_.call(resetmsg);
			ROS_WARN("[%s] Soft threshold reached, reset required", this->name().c_str());
		}

		if(this->has_reset_on_hard_ && (nstate == cybathlon::InputState::HardLeft ||
										nstate == cybathlon::InputState::HardRight)) {
			this->srv_reset_.call(resetmsg);
			ROS_WARN("[%s] Hard threshold reached, reset required", this->name().c_str());
		}

		// Update the return state
		rstate = nstate;
	}

	return rstate;
}

void DoubleThresholdWheel::on_receive_neuroprediction(const rosneuro_msgs::NeuroOutput& msg) {
	
	int refclass = this->classes_.at(0);
	int refclassIdx;
	bool class_not_found = false;
	std::vector<int> msgclasses = msg.decoder.classes;

	// First: check that the incoming classes are the ones provided
	for(auto it = msgclasses.begin(); it != msgclasses.end(); ++it) {
		auto it2 = std::find(this->classes_.begin(), this->classes_.end(), *it);
		if(it2 == this->classes_.end()) {
			class_not_found = true;
			break;
		}
	}

	// Second: find the index of the refclass
	if(class_not_found == true) {
		this->has_new_input_ = false;
		ROS_WARN_THROTTLE(5.0f, "[%s] The incoming neurooutput message" 
								"does not have the provided classes", 
								this->name().c_str());
		return;
	}

	auto it = std::find(msgclasses.begin(), msgclasses.end(), refclass);

	if(it != msgclasses.end()) {
		refclassIdx 		 = it - msgclasses.begin();
		this->ctr_input_     = msg.softpredict.data.at(refclassIdx);
		this->has_new_input_ = true;
	} else {
		this->has_new_input_ = false;
		ROS_WARN_THROTTLE(5.0f, "[%s] Cannot find class %d in the incoming message", 
								this->name().c_str(), refclass);
	}


}

void DoubleThresholdWheel::on_receive_artifact_event(const rosneuro_msgs::NeuroEvent& msg) {
	
	cybathlon::Artifact artevt = cybathlon::to_artifact(msg);

	switch(artevt) {
		case cybathlon::Artifact::Ocular:
			ROS_DEBUG_NAMED("debug", "[%s] Artifact received: Artifact::%s", 
							this->name().c_str(), 
							cybathlon::to_string(artevt).c_str());
			this->show_artifact();
			break;
		case cybathlon::Artifact::EndOcular:
			ROS_DEBUG_NAMED("debug", "[%s] Artifact received: Artifact::%s", 
							this->name().c_str(), 
							cybathlon::to_string(artevt).c_str());
			this->hide_artifact();
			break;
		default:
			break;
	}
}


void DoubleThresholdWheel::on_receive_game_event(const rosneuro_msgs::NeuroEvent& msg) {

	std_srvs::Empty resetmsg;
	GameTask task = cybathlon::to_gametask(msg);

	if(task == cybathlon::GameTask::End || task == cybathlon::GameTask::Undefined) 
		return;

	// Set the new task config
	this->config_ = this->get_task_config(task);

	// Update the wheel parameters with the new configuration
	this->set_wheel_parameters(*(this->config_));

	// Update dynamic reconfigure
	this->recfg_srv_->updateConfig(*(this->config_));

	// Ask for reset
	this->srv_reset_.call(resetmsg);
		
	ROS_DEBUG_NAMED("debug", "[%s] Changed parameters for task %s", 
					this->name().c_str(), 
					cybathlon::to_string(task).c_str());
}

void DoubleThresholdWheel::set_soft_threshold_left(double threshold) {
	double input_c, dist_c;

	input_c = (this->input_max_ + this->input_min_)/2.0f;
	dist_c  = std::fabs(input_c-threshold);

	this->soft_threshold_left_ = input_c + dist_c;
	this->soft_left_->rotate(this->input2angle(this->soft_threshold_left_), 0.0f, 0.0f);
}

void DoubleThresholdWheel::set_soft_threshold_right(double threshold) {
	double input_c, dist_c;

	input_c = (this->input_max_ + this->input_min_)/2.0f;
	dist_c  = std::fabs(input_c-threshold);

	this->soft_threshold_right_ = input_c - dist_c;
	this->soft_right_->rotate(this->input2angle(this->soft_threshold_right_), 0.0f, 0.0f);
}

void DoubleThresholdWheel::set_hard_threshold_left(double threshold) {
	double input_c, dist_c;

	input_c = (this->input_max_ + this->input_min_)/2.0f;
	dist_c  = std::fabs(input_c-threshold);

	this->hard_threshold_left_ = input_c + dist_c;
	this->hard_left_->rotate(this->input2angle(this->hard_threshold_left_), 0.0f, 0.0f);
}

void DoubleThresholdWheel::set_hard_threshold_right(double threshold) {
	double input_c, dist_c;

	input_c = (this->input_max_ + this->input_min_)/2.0f;
	dist_c  = std::fabs(input_c-threshold);

	this->hard_threshold_right_ = input_c - dist_c;
	this->hard_right_->rotate(this->input2angle(this->hard_threshold_right_), 0.0f, 0.0f);
}


void DoubleThresholdWheel::on_request_reconfigure(cybathlon_config_wheel& config, uint32_t level) {

	if( std::fabs(config.soft_threshold_left - this->soft_threshold_left_) > 0.00001) {
		this->set_soft_threshold_left(config.soft_threshold_left);
		ROS_WARN("[%s] Changed soft threshold left to: %f", this->name().c_str(), 
															config.soft_threshold_left);
	}
	
	if( std::fabs(config.soft_threshold_right - this->soft_threshold_right_) > 0.00001) {
		this->set_soft_threshold_right(config.soft_threshold_right);
		ROS_WARN("[%s] Changed soft threshold right to: %f", this->name().c_str(), 
															 config.soft_threshold_right);
	}
	
	if( std::fabs(config.hard_threshold_left - this->hard_threshold_left_) > 0.00001) {
		this->set_hard_threshold_left(config.hard_threshold_left);
		ROS_WARN("[%s] Changed hard threshold left to: %f", this->name().c_str(), 
															config.hard_threshold_left);
	}
	
	if( std::fabs(config.hard_threshold_right - this->hard_threshold_right_) > 0.00001) {
		this->set_hard_threshold_right(config.hard_threshold_right);
		ROS_WARN("[%s] Changed hard threshold right to: %f", this->name().c_str(), 
															 config.hard_threshold_right);
	}

	if(config.reset_on_soft != this->has_reset_on_soft_) {
		this->has_reset_on_soft_ =  config.reset_on_soft;
		ROS_WARN("[%s] Changed reset on soft to: %s", this->name().c_str(),
				  this->has_reset_on_soft_ ? "true" : "false");
	}
	
	if(config.reset_on_hard != this->has_reset_on_hard_) {
		this->has_reset_on_hard_ =  config.reset_on_hard;
		ROS_WARN("[%s] Changed reset on hard to: %s", this->name().c_str(),
				  this->has_reset_on_hard_ ? "true" : "false");
	}

	*(this->config_) = config;
}

}
