/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/CalibrateIntakeRotate.h"
#include "RobotMap.h"
#include <frc/Preferences.h>
#include <iostream>

CalibrateIntakeRotate::CalibrateIntakeRotate() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  this->SetRunWhenDisabled(true);
}

// Called just before this Command runs the first time
void CalibrateIntakeRotate::Initialize() {
  // Get intake current position, then set offsets
  const int currentPosition = RobotMap::rotateLeftMotor->GetSelectedSensorPosition();
  frc::Preferences::GetInstance()->PutInt("Intake.position.base", currentPosition);
  std::cout << "*** CalibrateIntakeRotate: Base is now " << currentPosition << "\n";
  SetTimeout(1);
}

// Called repeatedly when this Command is scheduled to run
void CalibrateIntakeRotate::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool CalibrateIntakeRotate::IsFinished() { return IsTimedOut(); }

// Called once after isFinished returns true
void CalibrateIntakeRotate::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CalibrateIntakeRotate::Interrupted() {}