/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Lift/Action.h"
#include "Lift/LiftDrive.h"

class SecondStep : public Action {
 public:
  SecondStep() = default;
  void Execute() override;
 private:
  LiftDrive liftDrive;
  double positionStartTime = -1.0;
  bool liftFinished = false;
  bool shiftedFrontToSwerve = false;
};
