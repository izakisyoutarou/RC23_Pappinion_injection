#pragma once
#include "ichigoplus/layer_controller/brush_motor_pos_vel_controller.hpp"

class BeltCollecter{

public:
	BeltCollecter(BrushMotorPosVelController &mc0, BrushMotorPosVelController mc1):mc0(mc0), mc1(mc1){};
	void start(float vel);
	void stop();

private:
	BrushMotorPosVelController &mc0;
	BrushMotorPosVelController &mc1;

};
