/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/CalibrateJackScrewsDownPosition.h"
#include "frc/Preferences.h"
#include "Robot.h"
#include <iostream>

CalibrateJackScrewsDownPosition::CalibrateJackScrewsDownPosition() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  this->SetRunWhenDisabled(true);
}

// Called just before this Command runs the first time
void CalibrateJackScrewsDownPosition::Initialize() {
  std::cout << "=== Start JackScrew Distance Calibration ===\n";
  frc::Preferences *prefs = frc::Preferences::GetInstance();
  auto end = Robot::driveBase->GetDriveEncoderPositions(); 
  DriveInfo<double> start {
    prefs->GetFloat("JackScrew.FL.base"),
    prefs->GetFloat("JackScrew.FR.base"),
    prefs->GetFloat("JackScrew.RL.base"),
    prefs->GetFloat("JackScrew.RR.base"),
  };
  DriveInfo<double> dist = end - start;

  prefs->PutFloat("JackScrew.FL.dist", dist.FL);
  prefs->PutFloat("JackScrew.FR.dist", dist.FR);
  prefs->PutFloat("JackScrew.RL.dist", dist.RL);
  prefs->PutFloat("JackScrew.RR.dist", dist.RR);

  SetTimeout(1);
  std::cout << "=== End JackScrew Distance Calibration ===\n";
}

// Called repeatedly when this Command is scheduled to run
void CalibrateJackScrewsDownPosition::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool CalibrateJackScrewsDownPosition::IsFinished() { return IsTimedOut(); }

// Called once after isFinished returns true
void CalibrateJackScrewsDownPosition::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CalibrateJackScrewsDownPosition::Interrupted() {}
