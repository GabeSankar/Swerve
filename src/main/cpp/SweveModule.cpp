#include "SwerveModule.h"
using namespace frc;
SwerveModule::SwerveModule(int RotatorPort, int DrivePort, int EncoderPort1, int EncoderPort2, bool reverseDirection):
RotatorMotor(RotatorPort),
DriveMotor(DrivePort),
RotatorEncoder(EncoderPort1, EncoderPort2,reverseDirection),
pid(KP, KI, KD)
{
	correction = 0.0;
	RotatorEncoder.Reset();
	RotatorEncoder.SetDistancePerPulse(0.8692152937);
	MotorIsForward = true;
}
void SwerveModule::ResetEncoder()
{
	RotatorEncoder.Reset();
}
double SwerveModule::GetCurrentPosition(){
	double angle = fmod((fmod(RotatorEncoder.GetDistance() , 360.0) + 360.0) , 360.0);
	if (angle == 0.0)
		angle = 360.0;
	return angle;
}

double SwerveModule::GetTurningEncoderPosition(){
	return RotatorEncoder.Get();
}

void SwerveModule::SetToVector(double speed, double angle,double throttle){
	if(speed*throttle >= 0.1 && speed*throttle <= 1){
		double TurningPower = pid.GetPID(SwerveModule::CalculateError(angle));
		correction = pid.GetPID(SwerveModule::CalculateError(angle));
		RotatorMotor.Set(TurningPower);
	}
	else{
		RotatorMotor.Set(0.0);
	}

	if(MotorIsForward && speed*throttle <= 1 && speed*throttle >= -1){
		
		DriveMotor.Set(speed*throttle);
	}
	else if(speed*throttle <= 1&& speed*throttle >= -1){
		
		DriveMotor.Set(-speed*throttle);
	}
}
double SwerveModule::CalculateError(double TargetAngle){

	double CDist = fmod((GetCurrentPosition()+(360.0-TargetAngle)),360.0);
	double error;
	double target = TargetAngle;
	if (target == 0)
		target = 360.0;
	if(CDist <= 90.0){
		error = CDist;
		MotorIsForward = true;
	}
	else if(CDist <= 270.0){
		error = CDist-180.0;
		MotorIsForward = false;
	}
	else if(CDist <= 360.0){
		error = CDist - 360.0;
		MotorIsForward = true;
	}
	else{
		error = 0.0;
	}

	return error;
}

