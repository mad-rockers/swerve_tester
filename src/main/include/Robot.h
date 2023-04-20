// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/ADIS16470_IMU.h>

#include "Ports.h"
#include "CustomController.h"
#include "swerve_system/SwerveDrive.h"

class Robot : public frc::TimedRobot {
 public:
  CustomController drivercontroller;
  SwerveDrive drivetrain;
  ADIS16470_IMU gyro;

  Robot() :
  drivercontroller(Ports::drivercontroller) 
  {}

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
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
};
