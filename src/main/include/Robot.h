// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include "SwerveModule.h"
#include "SwerveKnO.h"
#include <frc/TimedRobot.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  double throttle;
 private:
  SmartDashboard* dash;
  SwerveModule RFMod;
	SwerveModule LFMod;
	SwerveModule RBMod;
	SwerveModule LBMod;
  Joystick StickL;
	Joystick StickR;
  ADXRS450_Gyro gyro;
  SwerveKnO KinematicsAndOdometry;
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  bool FieldOriented;
};
