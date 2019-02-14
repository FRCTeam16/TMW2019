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
  bool IsRunning();

 private:
  enum class LiftState { kNone, kFirst, kSecond, kThird, kFinished };
  LiftState currentState = LiftState::kNone;
  std::unique_ptr<Action> currentAction;
};
