#pragma once

#include <frc/Encoder.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/CounterBase.h>
#include "PID.h"
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/controller/PIDController.h>
#include <cmath>
#include <frc/controller/ProfiledPIDController.h>
#include <units/angle.h>
#include <frc/geometry/Translation2d.h>
#include <units/base.h>
#include <units/length.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <units/angular_velocity.h>
#include "Constants.h"
#include <frc/AnalogPotentiometer.h>
#include <frc/AnalogInput.h>


using namespace frc;
class SwerveModule{
private:
	Spark RotatorMotor;
	Spark DriveMotor;
	AnalogPotentiometer RotatorEncoder;
	//AnalogInput analogIn;
	PID pid;
	
	bool MotorIsForward;
	double correction;
	void SetDrivePower(double power);
	double CalculateError(double TargetAngle);
public:
	SwerveModule(int RotatorPort, int DrivePort, int EncoderPort1, bool reverseDirection);
	
	double drive;
	double rotate;

	double setpointvelocity;
	void ResetEncoder();
	double GetCurrentPosition();
	double GetTurningEncoderPosition();
	double GetPEIDCorrection();
	void SetToVector(frc::SwerveModuleState& state);
	//Using Provided PID controllers take note to change later
  	frc2::PIDController DrivePID{KDp,KDi,KDd};
	frc::ProfiledPIDController<units::radians> turningPID{KRp,KRi,KRd,
	  													{MaxOmegaPerWheel*1_rad_per_s,MaxAlphaPerWheel*1_rad_per_s/1_s}};
	
	//FeedForward fix values
	frc::SimpleMotorFeedforward<units::meters> driveFF{1_V,3_V/1_mps};
	frc::SimpleMotorFeedforward<units::radians> rotatorFeedForward{1_V,.5_V/1_rad_per_s};
};
