#include "SwerveModule.h"
using namespace frc;
SwerveModule::SwerveModule(int RotatorPort, int DrivePort, int EncoderPort1, bool reverseDirection):
RotatorMotor(RotatorPort),
DriveMotor(DrivePort),
RotatorEncoder(EncoderPort1,360,0),
pidController(1,0,0)

{
	
	correction = 0.0;
	//Limits PID range -pi and pi(find out why these numbers are specified(read on internet))
	//Look over the range
    pidController.EnableContinuousInput(-2*3.1415926,2*3.1415926);
	MotorIsForward = true;
}
//ANGLE RETURNED IN RADIANS!!!!!!!!!!!!!!!!
double SwerveModule::GetCurrentPosition(){
	double angle = RotatorEncoder.Get();
	if (angle == 0.0)
		angle = 360.0;
	return angle*(M_PI/180);
}

double SwerveModule::GetTurningEncoderPosition(){
	return RotatorEncoder.Get();
}

//Input to motor state
void SwerveModule::SetToVector(frc::SwerveModuleState& state){
	//Optimised state to stop from spinning more than pi/2 radians
	auto optimizedstate = state.Optimize(state,(SwerveModule::GetCurrentPosition()*1_rad));
	auto driveOut = (state.speed*(1/(MAXSPEED*1_mps)));
	//auto rotOut = turningPID.Calculate(SwerveModule::GetCurrentPosition()*1_rad);
	double setpoint = state.angle.Radians().value()*(MAXVOLTAGE * 0.5) + (MAXVOLTAGE * 0.5); // Optimization offset can be calculated here.
    if (setpoint < 0) {
        setpoint = MAXVOLTAGE + setpoint;
    }
    if (setpoint > MAXVOLTAGE) {
        setpoint = setpoint - MAXVOLTAGE;
    }
	double rot = std::clamp(pidController.Calculate(SwerveModule::GetCurrentPosition(), setpoint), -1.0, 1.0);
   
	DriveMotor.Set(driveOut);
	RotatorMotor.Set(rot);
}
//Not used, currently using Optimize function to avoid spining over 90 degrees 
/*
double SwerveModule::CalculateError(double TargetAngle){

	double CDist = fmod((GetCurrentPosition()+((2*M_PI)-TargetAngle)),(2*M_PI));
	double error;
	double target = TargetAngle;
	if (target == 0)
		target = (2*M_PI);
	if(CDist <= (M_PI_2)){
		error = CDist;
		MotorIsForward = true;
	}
	else if(CDist <= (3*M_PI_2)){
		error = CDist-180.0;
		MotorIsForward = false;
	}
	else if(CDist <=  (2*M_PI)){
		error = CDist -  (2*M_PI);
		MotorIsForward = true;
	}
	else{
		error = 0.0;
	}

	return error;
}
*/
