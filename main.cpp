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
#include "layer_controller/Rappinion_injection.hpp"
#include "ichigoplus/layer_controller/photo_interrputer.hpp"

// circuit
#include "ichigoplus/layer_driver/circuit/emergency.hpp"
#include "ichigoplus/layer_driver/circuit/sbdbt.hpp"
#include "ichigoplus/layer_driver/circuit/can_motor_driver.hpp"
#include "ichigoplus/layer_driver/circuit/can_encoder.hpp"
#include "ichigoplus/layer_driver/circuit/can_digital.hpp"

// device
#include "stm32f4xx.h"
#include "layer_driver/device/pin.hpp"

// using
using encoder::Enc0;
using encoder::Enc1;
using encoder::Enc2;
using encoder::Enc3;
using TraVelPlanner = velocity_planner::trapezoidal_velocity_planner::TrapezoidalVelocityPlanner;
using TraVelPlannerLimit = velocity_planner::trapezoidal_velocity_planner::Limit_t;
using PhotoInt = photo_interrputer::PhotoInterrputerCanDigitalPinInterface;

int main(){
	// cycle period [ms]
	constexpr int ctrl_period = 2;
	constexpr int disp_period = 100;

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
	Can1 can1; can1.setup();
	//canは基盤同士を繋いでいる通信機能

	//candigital
	CanDigital canDigital000(can0,0,0);
	//使用するPhotoInterrputerの数に応じで増やす、今回は1つなのでcan0のみでok、複数使いたい場合はオブジェクト名をcanDigital000_1などにすること

	//PhotoInterrputer
	PhotoInt rappinion_photo(canDigital000.pin0, PhotoInt::SetupType::in);

	encoder::CanEncoder canEnc1(can0,0,ctrl_period); canEnc1.setup(); canEnc1.cpr(4000);
	//今回はcanの0番ポートに繋がっている
	//canEncは通信先の基盤に繋がっているエンコーダー
	//can0は基盤が繋がってるcanのポート、0はcan0が繋がっている基盤の先のエンコーダーのポート番号、ctrl_periodは周期
	//今回はcan0はマクソンのポート、can1はエンコーダーのポート
	//canEnc.cprは500*4で2000…500の部分はエンコーダーによって変わってくる

	encoder::FilteredEncoder filEnc1(canEnc1); filEnc1.setup(); filEnc1.rev(true);
	//FilteredEncoderのクラスの中のオブジェクト化したfilEnc0クラスの引数にcanEnc0を入れている,エンコーダーの向きが逆の時はfilEnc0.rev(false);のfalseをtrueに変える
	

	//CanMoterDriver
	CanMotorDriver canMd0(can0,0); canMd0.outRev(false); canMd0.currentLimit(CanMotorDriver::LIMIT_AVG, 100.f);
	//can通信方式なのでcanMd,srcのときはlapmoterだからMd
	//モーターを逆回転したいときはcanMd0.outRev(false);のfalseをtrueに変える

	//tvplanner
	const TraVelPlannerLimit tvplimit(M_PI*100000.f, M_PI*10000.f,M_PI*100000.f,M_PI*1000.f);
	//速度、加速度制限はここで行う、左から位置　速度　加速度　減速度

	TraVelPlanner tvp0(tvplimit);
	//siso_controlleer
	siso_controller::PositionPid pid0; pid0.gain(0.1f, 0.0f, 0.f);
	//pid制御でgain調整左からのp制御,i制御,d制御

	console::PidGain pidGain0; pidGain0.add("g0", pid0); cons.addCommand(pidGain0);

	//BrushMotorPosVelController
	BrushMotorPosVelController mc0(canMd0, filEnc1, tvp0, pid0); mc0.rotateRatio(1.f,1.f); mc0.limitDuty(-0.7f,0.7f);
	//クラスはオブジェクト化しないと使うことはできないオブジェクト名はその場で自由に決めることができる,今回はmc0がオブジェクト名
	//クラスの中でクラスを使うときはオブジェクト化したクラスを引数に入れる
	//この場合はBrushMotorPosVelControllerクラスの中のmc0クラスをオブジェクト化し、その引数にcanMd0, filEnc0, tvp0, pid0を入れている
	//今回の機構はモーターを2つ使っているが基板のmdポートが1つなのでmd関数も1つでいい
	mc0.commandName("mc0"); cons.addCommand(mc0);

	//other
	Rappinion_injection rappinion_injection(mc0, rappinion_photo);//クラス名　オブジェクト名（引数）

	// Cycle Timer
	Timer ctrlCycle;
	ctrlCycle(ctrl_period, true);
	Timer dispCycle;
	dispCycle(disp_period, true);

	// CycleChecker
	CycleChecker cycleChecker(ctrl_period);

	//CycleCounter
	cycle_once_checker::CycleCounter cycleCounter;

	// ExecuteFunction(add func)
	exeFunc.addFunc("reset", [&]{ NVIC_SystemReset();});
	exeFunc.addFunc("cal",[&]{rappinion_injection.calbiration();});
	exeFunc.addFunc("inject", [&]{ rappinion_injection.inject();});
	//posは現在の値、currentTargetPosはTraVelPlannerによって計算された理想の値、currentTargetPosにposを合わせる感じで調整する
	//射出のコマンドを打つと同時にprintf関数でposとcurrentTargetPosを動ことで、posの値とcurrentTargetPosの値の差を見ながらgainの値を調整する
	exeFunc.addFunc("back", [&]{ rappinion_injection.back();});
	exeFunc.addFunc("mc0", [&]{ mc0.duty(0.2);});   //モーターの回転方向を見たいときは mc0 duty 0.1と入力する
	//exeFunc.addFunc("duty",[&]{mc0.duty(0.4); mc1.duty(-0.4);});//モーターを回転させる際無限ループにfuncを入れないこと…func duty だけ　入力する
	//無限ループさせたいときは最後に1文字追加する…func duty a　など
	exeFunc.addFunc("enc", [&]{ printf("%f\n",filEnc1.radian());});
	exeFunc.addFunc("photostart", [&]{ rappinion_photo.startReading(2);});
	exeFunc.addFunc("photo", [&]{ printf("%d\n",rappinion_photo.read());});
	exeFunc.addFunc("pos",[&]{printf("%f\n",mc0.pos());});
	exeFunc.addFunc("calentpos",[&]{printf("%f\n",mc0.currentTargetPos());});

	//setusdf
	int error=0;
	mc0.setup();
	rappinion_photo.startReading(2);
	error += rappinion_injection.setup();
	error += canDigital000.setup();
	//canDigital000はメインでセットアップ
	printf("error:%d\n",error);

	//encの値を見るときはprintfの中に書く
	// main loop




	while(1){
		emergency.cycle();

		if(ctrlCycle()) {
			cycleChecker.cycle();
			cycleCounter.cycle();
			rappinion_injection.cycle();
			canDigital000.cycle();
			//PhotoInterrputer(rappinion_photo)にはサイクルがないためcanDigital000単体でサイクルをかけることでphotoの値が見れる。
			//forCons.printf("mc_vel:%f\n",mc0.duty());
			mc0.cycle();
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
