/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "networktables/NetworkTable.h"

struct SceneInfo {
  bool hasTarget;
  double xOffset;
  double yOffset;
  double targetArea;
  double skew;
  double latency;
};

/**
 * System for handling interaction with vision system.
 * 
 * @see http://docs.limelightvision.io/en/latest/networktables_api.html
 */
class Limelight {
 public:
 enum class CameraMode { Unknown = -1, ImageProcessing = 0, DriverCamera = 1 };

  Limelight();

  SceneInfo GetScene() const;
  
  int GetCurrentPipeline() const;
  void SelectPipeline(const int pipelineNumber);
  
  CameraMode GetCameraMode() const;
  void SetCameraMode(CameraMode visionMode);
  CameraMode ToggleCameraMode();

  /** Uses angle to target object to determine distance **/
  double CalculateDistance(double heightToCamera, double heightToTarget) const;

private:
  std::shared_ptr<NetworkTable> dataTable;
};
