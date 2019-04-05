/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Action.h"
#include "Subsystems/JackScrews.h"

class FirstStep : public Action {
 public:
  FirstStep();
  void Execute() override;
  bool IsFinished() override { return finished; }
 private:
  bool finished = false;
  bool firstAfterShifting = true;
  bool crawlerStarted = false;
};
