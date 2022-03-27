// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

Robot::Robot():
  RFMod(4, 0, 0, 1, false),
	LFMod(6, 2, 6, 7, false),
	RBMod(5, 1, 2, 3, false),
	LBMod(7, 3, 4, 5, false),
  StickL(0),
  StickR(1),
  gyro(),
  //Multiplied by 1 degree to transfer the Unit to the non Unit
  KinematicsAndOdometry(0_m,0_m,0_deg, (gyro.GetAngle()*1_deg))
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
 
  KinematicsAndOdometry.SwerveOdometryGetPose(gyro.GetAngle()*1_deg);
  if(FieldOriented){
  KinematicsAndOdometry.FieldRelativeKinematics((StickL.GetX()*velocityConstant*1_mps),(StickL.GetY()*velocityConstant*1_mps),
                                               (StickR.GetX()*rotationConstant*1_rad_per_s),(gyro.GetAngle()*1_deg));
  }else{
  KinematicsAndOdometry.notFieldRelativeKinematics((StickL.GetX()*velocityConstant*1_mps),(StickL.GetY()*velocityConstant*1_mps),(StickR.GetX()*rotationConstant*1_rad_per_s));
  }
  LFMod.SetToVector(KinematicsAndOdometry.motorDataMatrix[0][0],KinematicsAndOdometry.motorDataMatrix[0][1],throttle);
  RFMod.SetToVector(KinematicsAndOdometry.motorDataMatrix[1][0],KinematicsAndOdometry.motorDataMatrix[1][1],throttle);
  LBMod.SetToVector(KinematicsAndOdometry.motorDataMatrix[2][0],KinematicsAndOdometry.motorDataMatrix[2][1],throttle);
  RBMod.SetToVector(KinematicsAndOdometry.motorDataMatrix[3][0],KinematicsAndOdometry.motorDataMatrix[3][1],throttle);

  dash -> PutNumber("DistanceX",KinematicsAndOdometry.PoseVector[0]);
  dash -> PutNumber("DistanceY",KinematicsAndOdometry.PoseVector[1]);
  dash -> PutNumber("Angle",KinematicsAndOdometry.PoseVector[2]);
  
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
