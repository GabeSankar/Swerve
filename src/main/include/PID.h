#pragma once
#include <frc/Timer.h>
class PID{
    private:
	//PID coefficients
	double kP;
	double kI;
	double kD;

	//values of PID
	double P;
	double I;
	double D;
	
    
    //Vars needed in PID calculations
    double CurrentError;
	double LastError;
	double ElapsedTime;
	double CurrentTime;
	double LastTime;
    bool FirstRun;

    //Timer for delta time
    frc::Timer PIDTimer;

    double Pcalc(double error);
	double Icalc(double error);
	double Dcalc(double error);
    
    
    public:
    //PID object
    PID(double kP, double kI, double kD);
    //Public get correction method
    double GetPID(double error);

};