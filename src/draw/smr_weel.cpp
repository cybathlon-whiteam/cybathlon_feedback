#include "cybathlon_feedback/smr_weel.h"

namespace rosneuro {


smr_weel::smr_weel(void) : p_nh_("~") {

	this->sub_neuro_ = this->nh_.subscribe("/integrator/neuroprediction", 1, &smr_weel::on_received_neuro_data, this);

	this->engine_ = new neurodraw::Engine("smr_dweel");
	this->engine_->on_keyboard(&smr_weel::on_keyboard_event, this);

	this->user_quit_ = false;

	// Bind dynamic reconfigure callback
  this->recfg_callback_type_ = boost::bind(&smr_weel::on_request_reconfigure, this, _1, _2);
	this->recfg_srv_.setCallback(this->recfg_callback_type_);

  // Bind dynamic reconfigure finals
  ros::NodeHandle node_handle_b("~finals_tresholds");
  dyncfg_exponential *recfg_srv_f_ = new dyncfg_exponential(node_handle_b);
  this->recfg_exponential_callback_type_ = boost::bind(&smr_weel::on_request_reconfigure_f, this, _1, _2);
  recfg_srv_f_->setCallback(this->recfg_exponential_callback_type_);

  // Set the service clients
	this->client = this->nh_.serviceClient<dynamic_reconfigure::Reconfigure>(
        "/navigation_controller/thresholds/set_parameters");

	this->integrator_client = this->nh_.serviceClient<dynamic_reconfigure::Reconfigure>(
        "/integrator/thresholds_margin/set_parameters");
}

smr_weel::~smr_weel(void) {

	if(this->engine_ != nullptr)
		delete this->engine_;

}

bool smr_weel::configure(
			std::vector<double> threshold_soft,
			std::vector<double> threshold_hard,
			std::vector<double> threshold_final
	) {
	this->input_min_ = 0.0f;
	this->input_max_ = 1.0f;

	this->angle_min_ = 0.0f;
	this->angle_max_ = 180.0f;
	
	this->probablility = thresholds_angle(input2angle(threshold_soft[0]) , input2angle(1 - threshold_soft[1])  );
	this->command_a = thresholds_angle(input2angle(threshold_soft[0]) , input2angle(1 - threshold_soft[1]) , neurodraw::Palette::blue);
	this->command_b = thresholds_angle(input2angle(threshold_hard[0]) , input2angle(1 - threshold_hard[1]) , neurodraw::Palette::red);
	this->cmd_f = thresholds_angle(input2angle(threshold_final[0]) , input2angle(1 - threshold_final[1])  );

	this->setup_scene();

	return true;
}

bool smr_weel::configure() {
  std::string tsts = "0.8, 0.8";
  std::string tsth = "0.9, 0.9";
  std::string tstf = "1.0, 1.0";

  ros::param::param("~threshold_soft",  this->string_thresholds_soft_,  tsts);
  ros::param::param("~threshold_hard",  this->string_thresholds_hard_,  tsth);
  ros::param::param("~threshold_final", this->string_thresholds_final_, tstf);

  this->thresholds_soft_  = this->string2vector_converter(this->string_thresholds_soft_);
  this->thresholds_hard_  = this->string2vector_converter(this->string_thresholds_hard_);
  this->thresholds_final_ = this->string2vector_converter(this->string_thresholds_final_);
  
  return this->configure(this->thresholds_soft_, this->thresholds_hard_, this->thresholds_final_);
}


void smr_weel::setup_scene(void) {
	
	this->ring_  = new neurodraw::Ring(0.8f, 0.15f, neurodraw::Palette::grey);
  this->middle_line_ = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::darkgrey);

	this->arc_   = new neurodraw::Arc(0.8f, 0.15f, 2.0 * M_PI / 3.0f, neurodraw::Palette::lightgrey);
	this->mline_ = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::green);
	
	this->arc_->rotate(30);
	this->mline_->move(0.0f, 0.725f);
  this->middle_line_->move(0.0f, 0.725f);

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
  this->engine_->add(this->middle_line_);

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
	double input;
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

	double b, a, xmax, xmin, angle;

	b    = this->angle_max_;
	a    = this->angle_min_;
	xmax = this->input_max_;
	xmin = this->input_min_;
	
	angle = (b-a) * ( (input - xmin) / (xmax - xmin) ) + a;

	return angle;

}

std::vector<double> smr_weel::string2vector_converter(std::string msg){
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

void smr_weel::on_request_reconfigure(cybathlon_feedback &config, uint32_t level) {
	this->thresholds_soft_  = {config.thsl, config.thsr};
  this->thresholds_hard_  = {config.thhl, config.thhr};

	this->configure(this->thresholds_soft_, this->thresholds_hard_, this->thresholds_final_);

	// SPAM THE CONFIGURATION TO THE CONTROLLER
	// TODO: SET A BETTER CODE
	// TODO: CHECK IF IT IS BETTER TO PUT THIS IN THE CONTORLLER AND NOT IN THE FEEDBACK
	dynamic_reconfigure::Config config_c;

	dynamic_reconfigure::DoubleParameter double_param;
    double_param.name = "thsl"; // Sostituisci con il nome del parametro da modificare
    double_param.value = config.thsl; // Sostituisci con il valore desiderato
    config_c.doubles.push_back(double_param);
	double_param.name = "thsr"; // Sostituisci con il nome del parametro da modificare
    double_param.value = config.thsr; // Sostituisci con il valore desiderato
    config_c.doubles.push_back(double_param);
	double_param.name = "thhl"; // Sostituisci con il nome del parametro da modificare
    double_param.value = config.thhl; // Sostituisci con il valore desiderato
    config_c.doubles.push_back(double_param);
	double_param.name = "thhr"; // Sostituisci con il nome del parametro da modificare
    double_param.value = config.thhr; // Sostituisci con il valore desiderato
    config_c.doubles.push_back(double_param);

	this->srv.request.config = config_c;
	this->client.call(this->srv);

}

void smr_weel::on_request_reconfigure_f(exponential_feedback &config, uint32_t level) {
  this->thresholds_final_  = {config.thfl, config.thfr};

	this->configure(this->thresholds_soft_, this->thresholds_hard_, this->thresholds_final_);

	// SPAM THE CONFIGURATION TO THE INTEGRATOR
	// TODO: SET A BETTER CODE
	// TODO: CHECK IF IT IS BETTER TO PUT THIS IN THE CONTORLLER AND NOT IN THE FEEDBACK
  dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::Config config_i;

	double_param.name = "thfl"; // Sostituisci con il nome del parametro da modificare
  double_param.value = config.thfl; // Sostituisci con il valore desiderato
  config_i.doubles.push_back(double_param);
	double_param.name = "thfr"; // Sostituisci con il nome del parametro da modificare
  double_param.value = config.thfr; // Sostituisci con il valore desiderato
  config_i.doubles.push_back(double_param);

	this->srv.request.config = config_i;
	this->integrator_client.call(this->srv);

}

}
