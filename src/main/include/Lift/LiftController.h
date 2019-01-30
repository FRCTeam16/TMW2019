/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Subsystems/JackScrews.h"
#include "Action.h"

class LiftController {
 public:
  LiftController();
  void Next();
  void Run();

 private:
  enum class LiftState { kNone, kLiftUp, kFirstDrive, kLastDrive, kFinished };
  LiftState currentState = LiftState::kNone;
  std::unique_ptr<Action> currentAction;
};
