// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
  SmartDashboard::PutNumber("Gyro Angle", double(gyro.GetAngle()));

  auto [fl, fr, bl, br] = drivetrain.GetModuleStates();
  SmartDashboard::PutNumber("Front Left Drive", double(fl.speed));
  SmartDashboard::PutNumber("Front Left Steer", double(fl.angle.Degrees()));
  SmartDashboard::PutNumber("Front Right Drive", double(fr.speed));
  SmartDashboard::PutNumber("Front Right Steer", double(fr.angle.Degrees()));
  SmartDashboard::PutNumber("Back Left Drive", double(bl.speed));
  SmartDashboard::PutNumber("Back Left Steer", double(bl.angle.Degrees()));
  SmartDashboard::PutNumber("Back Right Drive", double(br.speed));
  SmartDashboard::PutNumber("Back Right Steer", double(br.angle.Degrees()));
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  if(r_driver.GetLeftBumper()) {
    drivetrain.SetX();
  }
  else {
    drivetrain.Drive(
      -r_driver.GetLeftX(),
      -r_driver.GetLeftY(),
      -r_driver.GetRightX(),
      //gyro.GetAngle());
      0_deg,
      false);
  }

  if(r_driver.GetRightBumper()) {
    while(r_driver.GetRightBumper()) {}
    gyro.Reset();
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
