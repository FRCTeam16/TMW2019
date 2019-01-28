/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/JackScrews.h"
#include "Robot.h"
#include "RobotMap.h"
#include "Subsystems/Drive/DriveBase.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <iostream>

JackScrews::JackScrews() : frontAxleSolenoid(RobotMap::frontAxleSolenoid), rearAxleSolenoid(RobotMap::rearAxleSolenoid)
{
  Preferences *prefs = Preferences::GetInstance();
  if (!prefs->ContainsKey("JackscrewSpeed")) {
    prefs->PutLong("JackscrewSpeed", 500);
  }
}

void JackScrews::Run()
{
  std::cout << "Jackscrews::Run(running = " << running << ") =>\n";

  if (running) {
    if (RunMode::kOpenLoop == currentRunMode) {
      for (auto const &wheel : Robot::driveBase->wheels) {
        wheel->UseOpenLoopDrive(openLoopSpeed);
      }
    }
    else {
      double rpm = Preferences::GetInstance()->GetLong("JackscrewSpeed");
      double speed = rpm * direction;
      cout << "  Setting jackscrew rpm speed: " << speed << "\n";
      
      for (auto const &wheel : Robot::driveBase->wheels) {
        wheel->UseClosedLoopDrive(speed);
      }
    }
  }

  frc::SmartDashboard::PutNumber("FL Velocity", Robot::driveBase->wheels[0]->GetDriveVelocity());
  frc::SmartDashboard::PutNumber("FL Outpuft Current", Robot::driveBase->wheels[0]->GetDriveOutputCurrent());

  std::cout << "Jackscrews::Run <=\n\n";
}

void JackScrews::SetAllSolenoidState(bool extend)
{
  SetFrontSolenoidState(extend);
  SetRearSolenoidState(extend);
}

void JackScrews::SetFrontSolenoidState(bool extend)
{
  frontAxleSolenoid->Set(extend);
  if (!extend)
  {
    running = false;
  }
}

void JackScrews::SetRearSolenoidState(bool extend)
{
  rearAxleSolenoid->Set(extend);
  if (!extend)
  {
    running = false;
  }
}

void JackScrews::SetExtendScrews(bool extend, bool running_)
{
  direction = extend ? 1 : -1;
  running = running_;
}

void JackScrews::RunOpenLoop(double speed)
{
  openLoopSpeed = speed;
  currentRunMode = RunMode::kOpenLoop;
  running = true;
}