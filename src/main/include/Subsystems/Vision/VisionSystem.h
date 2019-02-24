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
#include "XOffsetController.h"
#include "Util/UtilityFunctions.h"
#include <iostream>


struct VisionInfo {
public:
  bool hasTarget = false;
  bool inThreshold = false;
  double xOffset = 0.0;
  double xSpeed = 0.0;
};

/**
 * Vision-assisted alignment and driving
 */
class VisionSystem : public SubsystemManager {
 public:
  VisionSystem();
  void Run() override;
  void ToggleCameraMode();
  std::shared_ptr<VisionInfo> GetLastVisionInfo();
  std::shared_ptr<Limelight> GetLimelight() { return limelight; }
private:
  std::shared_ptr<Limelight> limelight;
  std::unique_ptr<XOffsetController> xoffsetController;
  std::unique_ptr<frc::PIDController> xoffPID;
  std::shared_ptr<VisionInfo> currentVisionInfo;
};
