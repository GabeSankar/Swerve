#include "PID.h"


PID::PID(double kP, double kI, double kD):
PIDTimer(){
    this->kP = kP;
	this->kI = kI;
	this->kD = kD;
	this->LastError = 0.0;
	this ->CurrentError = 0.0;
	this ->ElapsedTime = 0.0;
	this ->LastTime = 0.0;
	this ->CurrentTime = 0.0;
	this ->P = 0.0;
	this ->I = 0.0;
	this ->D = 0.0;
    this -> FirstRun = true;
}
double PID::Pcalc(double error){
    P = kP*error;
	return P;
}
double PID::Icalc(double error){
    I = kI*0.5*(LastError + error)*ElapsedTime;
    return I;
}
double PID::Dcalc(double error){
    D = kD*((error-LastError)/ElapsedTime);
    return D;
}
double PID::GetPID(double error){
    if(FirstRun){
		PIDTimer.Start();
		frc::Wait(0.05_s);
		FirstRun = false;
	}
	CurrentTime = PIDTimer.Get().value()*1000;
	ElapsedTime = PIDTimer.Get().value()-LastTime;
	LastTime = CurrentTime;

	Pcalc(error);
	
	Icalc(error);
	Dcalc(error);
	LastError = error;
	return P+I+D;
}