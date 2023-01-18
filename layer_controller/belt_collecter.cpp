#include "belt_collecter.hpp"

void BeltCollecter::start(float vel){
	mc0.vel(vel);
}

void BeltCollecter::stop(){
	mc0.vel(0.f);
}
