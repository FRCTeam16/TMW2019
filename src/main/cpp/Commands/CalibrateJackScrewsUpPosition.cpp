/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/CalibrateJackScrewsUpPosition.h"
#include "Robot.h"
#include "frc/Preferences.h"
#include <iostream>

CalibrateJackScrewsUpPosition::CalibrateJackScrewsUpPosition() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  SetRunWhenDisabled(true);
}

// Called just before this Command runs the first time
void CalibrateJackScrewsUpPosition::Initialize() {
  std::cout << "=== Start JackScrew Baseline Calibration ===\n";
  frc::Preferences *prefs = frc::Preferences::GetInstance();
  auto encoders = Robot::driveBase->GetDriveEncoderPositions(); 
  prefs->PutInt("JackScrew.FL.base", encoders.FL);
  prefs->PutInt("JackScrew.FR.base", encoders.FR);
  prefs->PutInt("JackScrew.RL.base", encoders.RL);
  prefs->PutInt("JackScrew.RR.base", encoders.RR);

  SetTimeout(1);
  std::cout << "=== End JackScrew Baseline Calibration ===\n";
  std::cout << "=== You should not run the jackscrew down distance calibration ===\n";
}

// Called repeatedly when this Command is scheduled to run
void CalibrateJackScrewsUpPosition::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool CalibrateJackScrewsUpPosition::IsFinished() { 
  return IsTimedOut();
}

// Called once after isFinished returns true
void CalibrateJackScrewsUpPosition::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CalibrateJackScrewsUpPosition::Interrupted() {}
