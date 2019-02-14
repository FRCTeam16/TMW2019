/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Subsystems/Drive/DriveInfo.h"


class LiftDrive {
public:
  LiftDrive();
  void DriveFront(double twist, double y, double x, bool useGyro);
  void DriveTank(double leftInput, double rightInput);
private:
  enum class DriveMode { kAll, kFront };
  DriveMode driveMode = DriveMode::kFront;
  void SetSteering(DriveInfo<double> setpoint);
  void SetDriveSpeed(DriveInfo<double> speed);

  const bool driveFront = true;
};
