#include "Rappinion_injection.hpp"
int Rappinion_injection::setup(){
	int i=0;
	i += mc0.setup();
	i += photo.setup();
	return i;
}

void Rappinion_injection::cycle(){
	//if文は1か0で判断している1は通る0は通らないためこの場合はtrueになれば自動的に入るようになっている
	//photoはreadしているときは1を返す（反応していない時）
	if(calbiration_flag==true){
		photo.startReading(2.f);
		if(process==0){
			if(photo.read()==1){
				mc0.vel(-0.5);
			}	
			else if(photo.read()==0){
				process++;
			}
		}else if(process==1){
			mc0.vel(0.5);
			if(photo.read()==1){
				process++;
			}
		}else if(process==2){
			mc0.vel(0.f);
			mc0.initPos(0.f);//posの初期化…つまりその位置を原点とする
			mc0.restart();//後で消す…そのうちinitposだけで良くなる
			photo.stopReading();
			calbiration_flag=false;
			process=0;
			printf("calibration_finish\n");
		}
		
	}
	mc0.cycle();
}

void Rappinion_injection::calbiration(){
	calbiration_flag=true;
	
}

void Rappinion_injection::inject(){
	mc0.pos(50.f);//posはTraVelPlanner関数によって上手く計算されるため速度は書かない
				//posは位置情報、引数に50と入れれば50の位置までモーターが回る
}

void Rappinion_injection::back(){
	mc0.pos(10.f);//posで0.fと入力すると今いる位置から初期設定した位置、つまりinitPosで指定した位置に戻る
}
