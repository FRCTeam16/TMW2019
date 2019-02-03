/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Lift/Action.h"

class SecondStep : public Action {
 public:
  SecondStep();
  void Execute() override;
  bool IsFinished() override { return finished; }
 private:
  bool finished = false;
  bool liftFinished = false;
  double positionStartTime = -1.0;
};
