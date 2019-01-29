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
#include "Subsystems/Drive/SwerveWheel.h"
#include <vector>

using namespace frc;

class JackScrews : public SubsystemManager
{
public:
  enum class ShiftMode { kDrive = false, kJackscrews = true };
  enum class LiftMode { kAll, kFront, kBack };
  
  JackScrews();
  void Run() override;

  void SetAllSolenoidState(ShiftMode shiftMode);
  void SetFrontSolenoidState(ShiftMode shiftMode);
  void SetRearSolenoidState(ShiftMode shiftMode);

  void SetLiftMode(LiftMode liftMode);
  void RunOpenLoop(double speed);

 private:
    std::shared_ptr<Solenoid> frontAxleSolenoid;
    std::shared_ptr<Solenoid> rearAxleSolenoid;
    std::vector<std::shared_ptr<SwerveWheel>> allWheels;
    std::vector<std::shared_ptr<SwerveWheel>> frontAxis;
    std::vector<std::shared_ptr<SwerveWheel>> rearAxis;

    bool running = false;
    bool direction = 1.0;

    LiftMode currentLiftMode = LiftMode::kAll;
    double openLoopSpeed = 0.0;
};
