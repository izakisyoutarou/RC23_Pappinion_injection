//application
#include "ichigoplus/layer_application/cycle_checker.hpp"
#include "ichigoplus/layer_application/timer.hpp"
#include "ichigoplus/layer_application/console.hpp"

//controller


//circuit
#include "ichigoplus/layer_driver/circuit/emergency.hpp"
#include "ichigoplus/layer_driver/circuit/sbdbt.hpp"

//device
#include "layer_driver/device/pin.hpp"


int main(){
	//cycle period [ms]
	constexpr int ctrl_period = 2;
	constexpr int disp_period = 100;

	//Emergency
	EmergencySwitch e_switch; e_switch.setupDigitalOut();
	EmergencyRead	e_read;	e_read.digitalRead();
	Emergency emergency(e_switch, e_read);
	emergency.setup();

	//LED
	Led0 led0;led0.setupDigitalOut();
	Led1 led1;led1.setupDigitalOut();
	Led2 led2;led2.setupDigitalOut();
	Led3 led3;led3.setupDigitalOut();
	Led4 led4;led4.setupDigitalOut();
	Led5 led5;led5.setupDigitalOut();
	ResetLed resetled;resetled.setupDigitalOut();

	//full color LED
	Red red;
	Blue blue;
	Green green;

	//cycle LED
	auto &cycleLed = green;
	green.setupDigitalOut();
	
	//LCD
	LCDBackLight lcdbl;lcdbl.setupDigitalOut();
	lcdbl.digitalHigh();
	I2c0 i2c;

	//Buzzer
	Buzzer buzzer; buzzer.setupPwmOut(2500.0,0.0);

	//Switch
	Sw0 sw0;sw0.setupDigitalIn();
	Sw1 sw1;sw1.setupDigitalIn();
	Sw2 sw2;sw2.setupDigitalIn();
	Sw3 sw3;sw3.setupDigitalIn();

	//Pwm
	Pwm0 pwm0;
	Pwm1 pwm1;
	Pwm2 pwm2;
	Pwm3 pwm3;
	Pwm4 pwm4;
	Pwm5 pwm5;

	//Varaible-Voltage(3.3V/5V)inDigital
	D0 d0;
	D1 d1;
	D2 d2;
	D3 d3;

	//5V digital
	D5v0 d5v0;
	D5v1 d5v1;
	D5v2 d5v2;
	D5v3 d5v3;

	//Analog, Digital
	A0 a0;
	A1 a1;
	A2 a2;
	A3 a3;
	A4 a4;
	A5 a5;
	A6 a6;
	A7 a7;

	//Serial
	Serial0 forCons;
	Serial1 serial1;
	Serial2 serial2;
	Serial3 serial3;
	Serial4 serial4;
	Serial5 serial5;

	//Console
	Console cons(forCons);
	cons.setup(115200);
	cons.setNewLine(Console::NEWLINE_CRLF);

	//Sbdbt
	Sbdbt psCon(serial5);
	psCon.setup();

	//Encoder
	Enc0 enc0;
	Enc1 enc1;
	Enc2 enc2;
	Enc3 enc3;

	//Can
	Can0 can0;
	Can1 can1;

	//Cycle Timer
	Timer ctrlCycle;
	ctrlCycle(ctrl_period, true);
	Timer dispCycle;
	dispCycle(disp_period, true);

	//CycleChecker
	CycleChecker cycleChecker(ctrl_period);
	
	//main loop
	while(1){
		emergency.cycle();

		if(ctrlCycle()) {
			cycleChecker.cycle();

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