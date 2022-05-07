#include "SwerveModule.h"
using namespace frc;
SwerveModule::SwerveModule(int RotatorPort, int DrivePort, int EncoderPort1, bool reverseDirection):
RotatorMotor(RotatorPort),
DriveMotor(DrivePort),
RotatorEncoder(EncoderPort1,360,0),
pid(KP, KI, KD)
{
	
	correction = 0.0;
	//Limits PID range -pi and pi(find out why these numbers are specified(read on internet))
	turningPID.EnableContinuousInput(-M_PI*1_rad,M_PI*1_rad);
	//RotatorEncoder.Reset();
	//RotatorEncoder.SetDistancePerPulse(0.8692152937);
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

	auto driveOut = (state.speed.value()*(1/(MAXSPEED)));

	auto rotOut = turningPID.Calculate((SwerveModule::GetCurrentPosition()*1_rad),
									   optimizedstate.angle.Radians());

	//units::volt_t rotatorFF = rotatorFeedForward.Calculate(turningPID.GetSetpoint().velocity);
	//setpointvelocity = (turningPID.GetSetpoint().velocity).value();
	
	units::volt_t rot =(units::volt_t{rotOut}/(MAXVOLTAGE*1_V));
	DriveMotor.Set(driveOut);
	RotatorMotor.SetVoltage(rot);
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
