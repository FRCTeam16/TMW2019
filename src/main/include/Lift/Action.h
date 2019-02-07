/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "frc/Timer.h"

class Action {
 public:
  virtual ~Action() = default;

    virtual bool IsFinished() { return finished; }
  void Run() {
    if (firstRun) {
      startTime = frc::Timer::GetFPGATimestamp();
    }
    this->Execute();
    
    if (firstRun) {
      firstRun = false;
    }
  }

 protected:
  virtual void Execute() = 0;
  double startTime = -1;
  bool finished = false;
  bool IsFirstRun() const { return firstRun; }

 private:
  bool firstRun = true;
};
