#pragma once

#include <frc/Encoder.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/CounterBase.h>
#include "PID.h"
#include "Constants.h"

using namespace frc;
class SwerveModule{
private:
	Spark RotatorMotor;
	Spark DriveMotor;
	Encoder RotatorEncoder;
	PID pid;
	bool MotorIsForward;
	double correction;
	void SetDrivePower(double power);
	double CalculateError(double TargetAngle);
public:
	SwerveModule(int RotatorPort, int DrivePort, int EncoderPort1, int EncoderPort2, bool reverseDirection);
	
	void ResetEncoder();
	double GetCurrentPosition();
	double GetTurningEncoderPosition();
	double GetPEIDCorrection();
	void SetToVector(double velocitymagnitude, double angle,double throttle);
};
