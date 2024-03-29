/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "ctre/Phoenix.h"
#include <map>
#include "Subsystems/SubsystemManager.h"

class IntakeRotate : public SubsystemManager{
 public:
  enum class IntakePosition { kStarting, kCargoShot, kRocketShot, kLevelOne, kCargoPickup, kFloor };

  IntakeRotate();
  void Init() override;
  void Run() override;
  void SetIntakePosition(IntakePosition position);
  IntakePosition GetIntakePosition();
  void Instrument() override;
  void SetPositionSpeed(double speed, bool flipMode); // flip mode: true for open loop, false for closed if not already closed
  void DisabledHoldCurrentPosition();
  void CalibrateHome();

 private:
  std::shared_ptr<WPI_TalonSRX> rotateLeft;
  std::shared_ptr<WPI_VictorSPX> rotateRight;
  std::map<IntakeRotate::IntakePosition, int> positionLookup;
  IntakePosition targetPosition = IntakePosition::kStarting;

  int targetPositionValue = 0;
  double computedTargetValue = 0; // includes base and offset
  double positionSpeed = 0.0; // testing
  bool positionControl = true;
  double rotateAngle = 0.0;

  // Startup Handling
  bool initializeFinished = false;
  int initializeScanCounts = 0;
  int kInitializeScanCountMax = 25;
};
