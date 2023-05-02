#ifndef CYBATHLON_FEEDBACK_DWEEL_BARS_H_
#define CYBATHLON_FEEDBACK_DWEEL_BARS_H_

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>

#include "cybathlon_feedback/smr_dweel.h"

namespace rosneuro {

class smr_dweel_bars : public smr_dweel {
  public:
    smr_dweel_bars(void);

    bool configure(
			std::vector<float> threshold_soft,
			std::vector<float> threshold_hard
	  );

    void update() override;

  protected:
    void setup_scene(void) override;
    void on_received_bar_data(const std_msgs::Float32MultiArray& msg);

    
  private:
    ros::Subscriber 	    sub_bar_;

    neurodraw::Rectangle* 	abar_;
		neurodraw::Rectangle* 	bbar_;
		
		neurodraw::Rectangle* 	aline_;
		neurodraw::Rectangle* 	bline_;

    float bar1_ = 0.0f;
    float bar2_ = 0.0f;
    
};

}

#endif
