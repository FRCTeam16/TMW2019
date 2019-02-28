/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Autonomous/Step.h"
#include "Subsystems/Intake.h"

class DoIntakeAction : public Step {
 public:
  enum class Action { kEjectHatch, kIntakeHatch };
  DoIntakeAction(Action action, double timeToRun);
  bool Run(std::shared_ptr<World> world) override;
private:
  double startTime = -1;
  const Action action;
  const double timeToRun;
};
