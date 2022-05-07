// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>


Robot::Robot():
  RFMod(7, 6, 7, false),
	LFMod(5, 4, 5, false),
	RBMod(1, 0, 1, true),
	LBMod(3, 2, 3, true),
  gyro(),
  //Multiplied by 1 degree to transfer the Unit to the non Unit
  KinematicsAndOdometry(0_m,0_m,0_rad, (gyro.GetAngle()*1_rad))
  {
    dash -> init();
  }

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  gyro.Reset();
  FieldOriented = true;
  throttle = 1;
  
}

void Robot::TeleopPeriodic() {
  //Apperently Xbox inputs or reversed compared to joystick
  //Note Deadband in Xbox controller
  const auto xSpeed = -xspeedLimiter.Calculate(frc::ApplyDeadband(controller.GetLeftX(),0.02)*MAXSPEED);
  const auto ySpeed = -yspeedLimiter.Calculate(frc::ApplyDeadband(controller.GetLeftY(),0.02)*MAXSPEED);
  const auto rotSpeed = -rotspeedLimiter.Calculate(frc::ApplyDeadband(controller.GetRightX(),0.02)*MAXOmega);
 
  KinematicsAndOdometry.SwerveOdometryGetPose(gyro.GetAngle()*1_rad);
  if(FieldOriented){
  KinematicsAndOdometry.FieldRelativeKinematics((xSpeed*1_mps),(ySpeed*1_mps),
                                               (rotSpeed*1_rad_per_s),(gyro.GetAngle()*1_rad));
  }else{
  KinematicsAndOdometry.notFieldRelativeKinematics((10_mps),(10_mps),(4_rad_per_s));
  //KinematicsAndOdometry.notFieldRelativeKinematics((StickL.GetX()*velocityConstant*1_mps),(StickL.GetY()*velocityConstant*1_mps),(0_rad_per_s));
  }
  /*
  dash->PutNumber("LeftFF",LFMod.setpointvelocity);
  dash->PutNumber("LeftFMotorSpeed",KinematicsAndOdometry.motorDataMatrix[0][0]);
   dash->PutNumber("LeftFMotorAngle",KinematicsAndOdometry.motorDataMatrix[0][1]);
   dash->PutNumber("RotVal",rotSpeed);
   dash->PutNumber("RotationIn",LFMod.rotate);
   dash->PutNumber("XVal",xSpeed);
   dash->PutNumber("YVal",ySpeed);
   */
  //dash->PutNumber("EncoderAngle",LBMod.GetTurningEncoderPosition());
  dash->PutNumber("LFPos",LFMod.GetCurrentPosition());
  dash->PutNumber("LBPos",LBMod.GetCurrentPosition());
  dash->PutNumber("RBPos",RBMod.GetCurrentPosition());
  dash->PutNumber("RFPos",RFMod.GetCurrentPosition());
  
  //dash->PutNumber("angle",KinematicsAndOdometry.frontLeft.angle.Radians().value());
  
  LFMod.SetToVector(KinematicsAndOdometry.frontLeft);
 RFMod.SetToVector(KinematicsAndOdometry.frontRight);
  LBMod.SetToVector(KinematicsAndOdometry.backLeft);
  RBMod.SetToVector(KinematicsAndOdometry.backRight);
/*
  dash -> PutNumber("DistanceX",KinematicsAndOdometry.PoseVector[0]);
  dash -> PutNumber("LeftFrontDrive",LFMod.drive);
  dash -> PutNumber("LeftFrontRotate",LFMod.rotate);
  dash -> PutNumber("sf",KinematicsAndOdometry.motorDataMatrix[0][0]);
  dash -> PutNumber("DistanceY",KinematicsAndOdometry.PoseVector[1]);
  dash -> PutNumber("Angle",KinematicsAndOdometry.PoseVector[2]);
  */
  
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
