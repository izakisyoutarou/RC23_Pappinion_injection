// application
#include "ichigoplus/layer_application/cycle_checker.hpp"
#include "ichigoplus/layer_application/cycle_once_checker.hpp"
#include "ichigoplus/layer_application/timer.hpp"
#include "ichigoplus/layer_application/console.hpp"
#include "ichigoplus/layer_application/execute_function.hpp"
#include "ichigoplus/layer_application/pid_gain.hpp"

// controller
#include "ichigoplus/layer_controller/brush_motor_pos_vel_controller.hpp"
#include "ichigoplus/layer_controller/trapezoidal_velocity_planner.hpp"
#include "ichigoplus/layer_controller/siso_controller.hpp"
#include "ichigoplus/layer_controller/position_pid.hpp"
#include "ichigoplus/layer_controller/filtered_encoder.hpp"
#include "layer_controller/belt_collecter.hpp"

// circuit
#include "ichigoplus/layer_driver/circuit/emergency.hpp"
#include "ichigoplus/layer_driver/circuit/sbdbt.hpp"
#include "ichigoplus/layer_driver/circuit/can_motor_driver.hpp"
#include "ichigoplus/layer_driver/circuit/can_encoder.hpp"

// device
#include "layer_driver/device/pin.hpp"

// using
using encoder::Enc0;
using encoder::Enc1;
using encoder::Enc2;
using encoder::Enc3;
using TraVelPlanner = velocity_planner::trapezoidal_velocity_planner::TrapezoidalVelocityPlanner;
using TraVelPlannerLimit = velocity_planner::trapezoidal_velocity_planner::Limit_t;

int main(){
	// cycle period [ms]
	constexpr int ctrl_period = 2;
	constexpr int disp_period = 10;

	// Emergency
	EmergencySwitch e_switch; e_switch.setupDigitalOut();
	EmergencyRead	e_read;	e_read.digitalRead();
	Emergency emergency(e_switch, e_read);
	emergency.setup();

	// LED
	Led0 led0;led0.setupDigitalOut();
	Led1 led1;led1.setupDigitalOut();
	Led2 led2;led2.setupDigitalOut();
	Led3 led3;led3.setupDigitalOut();
	Led4 led4;led4.setupDigitalOut();
	Led5 led5;led5.setupDigitalOut();
	ResetLed resetled;resetled.setupDigitalOut();

	// Full Color LED
	Red red;
	Blue blue;
	Green green;

	// Cycle LED
	auto &cycleLed = green;
	green.setupDigitalOut();
	
	// LCD
	LCDBackLight lcdbl;lcdbl.setupDigitalOut();
	lcdbl.digitalHigh();
	I2c0 i2c;

	// Buzzer
	Buzzer buzzer; buzzer.setupPwmOut(2500.0,0.0);

	// Switch
	Sw0 sw0;sw0.setupDigitalIn();
	Sw1 sw1;sw1.setupDigitalIn();
	Sw2 sw2;sw2.setupDigitalIn();
	Sw3 sw3;sw3.setupDigitalIn();

	// Pwm
	Pwm0 pwm0;
	Pwm1 pwm1;
	Pwm2 pwm2;
	Pwm3 pwm3;
	Pwm4 pwm4;
	Pwm5 pwm5;

	// Varaible-Voltage(3.3V/5V)inDigital
	D0 d0;
	D1 d1;
	D2 d2;
	D3 d3;

	// 5V digital
	D5v0 d5v0;
	D5v1 d5v1;
	D5v2 d5v2;
	D5v3 d5v3;

	// Analog, Digital
	A0 a0;
	A1 a1;
	A2 a2;
	A3 a3;
	A4 a4;
	A5 a5;
	A6 a6;
	A7 a7;

	// Serial
	Serial0 forCons;
	Serial1 serial1;
	Serial2 serial2;
	Serial3 serial3;
	Serial4 serial4;
	Serial5 serial5;

	// Console
	Console cons(forCons);
	cons.setup(115200);
	cons.setNewLine(Console::NEWLINE_CRLF);

	// ExecuteFunction
	ExecuteFunction exeFunc("func");
	cons.addCommand(exeFunc);

	// Sbdbt
	Sbdbt psCon(serial5);
	psCon.setup();

	// Encoder
	Enc0 enc0;
	Enc1 enc1;
	Enc2 enc2;
	Enc3 enc3;

	// Can
	Can0 can0; can0.setup();
	encoder::CanEncoder canEnc0(can0,0,ctrl_period); canEnc0.setup(); canEnc0.cpr(400);
	encoder::FilteredEncoder filEnc0(canEnc0); filEnc0.setup(); filEnc0.rev(true);
	Can1 can1; can1.setup();
	encoder::CanEncoder canEnc1(can1,0,ctrl_period); canEnc1.setup(); canEnc1.cpr(400);
	encoder::FilteredEncoder filEnc1(canEnc1); filEnc1.setup(); filEnc1.rev(true);

	//CanMoterDriver
	CanMotorDriver canMd0(can0,0); canMd0.outRev(true); canMd0.currentLimit(CanMotorDriver::LIMIT_AVG, 100.f);
	CanMotorDriver canMd1(can1,0); canMd1.outRev(true); canMd1.currentLimit(CanMotorDriver::LIMIT_AVG, 100.f);

	//tvplanner
	const TraVelPlannerLimit tvplimit(M_PI*1000000.f, M_PI*23.f,M_PI*5.f,M_PI*5.f);
	TraVelPlanner tvp0(tvplimit);
	TraVelPlanner tvp1(tvplimit);

	//siso_controlleer
	siso_controller::PositionPid pid0; pid0.gain(1.f, 0.f, 0.f);
	console::PidGain pidGain0; pidGain0.add("g0", pid0); cons.addCommand(pidGain0);
	siso_controller::PositionPid pid1; pid1.gain(1.f, 0.f, 0.f);
	console::PidGain pidGain1; pidGain1.add("g1", pid0); cons.addCommand(pidGain1);

	//BrushMotorPosVelController
	BrushMotorPosVelController mc0(canMd0, filEnc0, tvp0, pid0); mc0.setup(); mc0.rotateRatio(1.f,1.f); mc0.limitDuty(-0.3f, 0.3f); mc0.outRev(true);
	mc0.commandName("mc0"); cons.addCommand(mc0);
	BrushMotorPosVelController mc1(canMd1, filEnc1, tvp1, pid1); mc1.setup(); mc1.rotateRatio(1.f,1.f); mc1.limitDuty(-0.3f, 0.3f); mc1.outRev(true);
	mc0.commandName("mc1"); cons.addCommand(mc1);

	//other
	BeltCollecter beltcollectet(mc0, mc1);

	// Cycle Timer
	Timer ctrlCycle;
	ctrlCycle(ctrl_period, true);
	Timer dispCycle;
	dispCycle(disp_period, true);

	// CycleChecker
	CycleChecker cycleChecker(ctrl_period);
	
	// CycleCounter
	cycle_once_checker::CycleCounter cycleCounter;

	// ExecuteFunction(add func)
	exeFunc.addFunc("reset", [&]{ NVIC_SystemReset(); });
	
	exeFunc.addFunc("start", [&]{ beltcollectet.start(3.14); });
	exeFunc.addFunc("stop", [&]{ beltcollectet.stop(); });

	// main loop
	while(1){
		emergency.cycle();

		if(ctrlCycle()) {
			cycleChecker.cycle();
			cycleCounter.cycle();
			canEnc0.cycle();
			canEnc1.cycle();
			canMd0.cycle();
			canMd1.cycle();
		}

		if(dispCycle()) {
			if(cycleChecker()){
				forCons.printf("cycle was delayed : %lld[ms]\n",cycleChecker.getMaxDelay());
				cycleChecker.reset();
			}
			cons.cycle();
			cycleLed.digitalToggle();
		}

	}

}
