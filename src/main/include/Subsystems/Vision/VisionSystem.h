/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Subsystems/SubsystemManager.h"
#include "frc/WPIlib.h"
#include "Limelight.h"
#include "RotateController.h"
#include <iostream>



/**
 * Test utilities for exercising vision-assisted alignment and driving
 */
class VisionSystem : public SubsystemManager {
 public:
  VisionSystem();
  void Run() override;
  void ToggleCameraMode();
private:
  std::shared_ptr<Limelight> limelight;
  std::unique_ptr<RotateController> rotate;
  std::unique_ptr<frc::PIDController> rotatePID;
  double calculateLockAngle(double gyro_);
};
