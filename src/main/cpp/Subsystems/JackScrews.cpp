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
  frontAxis.push_back(Robot::driveBase->wheels[0]);
  frontAxis.push_back(Robot::driveBase->wheels[1]);
  rearAxis.push_back(Robot::driveBase->wheels[2]);
  rearAxis.push_back(Robot::driveBase->wheels[3]);

  allWheels.reserve(frontAxis.size() + rearAxis.size());
  allWheels.insert(allWheels.end(), frontAxis.begin(), frontAxis.end());
  allWheels.insert(allWheels.end(), rearAxis.begin(), rearAxis.end());
}

void JackScrews::Run()
{
  std::cout << "Jackscrews::Run(running = " << running << ") =>\n";

  if (running)
  {
    std::vector<std::shared_ptr<SwerveWheel>> wheels;
    switch(currentLiftMode) {
      case LiftMode::kFront:
        wheels = frontAxis;
        break;
      case LiftMode::kBack:
        wheels = rearAxis;
        break;
      default:
        wheels = allWheels;
    }

    for (auto const &wheel : wheels) {
      wheel->UseOpenLoopDrive(openLoopSpeed);
    }
  }
  std::cout << "Jackscrews::Run <=\n\n";
}

void JackScrews::SetAllSolenoidState(ShiftMode shiftMode) {
  SetFrontSolenoidState(shiftMode);
  SetRearSolenoidState(shiftMode);
  running = static_cast<bool>(shiftMode);
}

void JackScrews::SetFrontSolenoidState(ShiftMode shiftMode) {
  frontAxleSolenoid->Set(static_cast<bool>(shiftMode));
}

void JackScrews::SetRearSolenoidState(ShiftMode shiftMode) {
  rearAxleSolenoid->Set(static_cast<bool>(shiftMode));
}

void JackScrews::SetLiftMode(LiftMode liftMode) {
  currentLiftMode = liftMode;
}

void JackScrews::RunOpenLoop(double speed) {
  openLoopSpeed = speed;
  running = true;
}