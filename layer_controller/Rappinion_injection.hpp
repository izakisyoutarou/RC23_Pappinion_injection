#pragma once
#include "ichigoplus/layer_controller/brush_motor_pos_vel_controller.hpp"
#include "ichigoplus/layer_controller/photo_interrputer.hpp"
#include "ichigoplus/layer_driver/circuit/can_encoder.hpp"

class Rappinion_injection{

public:
	Rappinion_injection(BrushMotorPosVelController &mc0, photo_interrputer::PhotoInterrputer &photo):mc0(mc0), photo(photo){};

	//BrushMotorPosVelControllerの中にfilencのクラスが入っているため書かなくていい
	void cycle();
	int setup();

	void inject();
	void back();
	void calbiration();
	
private:
	BrushMotorPosVelController &mc0;
	photo_interrputer::PhotoInterrputer &photo;

	bool calbiration_flag=false;
	int process=0;

};
