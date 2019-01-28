/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/WPILib.h>
#include <frc/Solenoid.h>

#include "SubsystemManager.h"

using namespace frc;

class JackScrews : public SubsystemManager
{
public:
  enum class ShiftMode { kDrive = false, kJackscrews = true };

  JackScrews();

  void Run() override;

  void SetAllSolenoidState(ShiftMode shiftMode);

  void ExtendClosedLoop(bool extend);
  void Stop() { running = false; }
  void RunOpenLoop(double speed);

private:
  std::shared_ptr<Solenoid> frontAxleSolenoid;
  std::shared_ptr<Solenoid> rearAxleSolenoid;

  enum class RunMode { kOpenLoop, kClosedLoop };

  enum RunMode currentRunMode = RunMode::kOpenLoop;
  bool running = false;
  bool direction = 1.0;
  double openLoopSpeed = 0.0;

  // TODO: Make public once axle control in place
  void SetFrontSolenoidState(ShiftMode shiftMode);
  void SetRearSolenoidState(ShiftMode shiftMode);
};
