/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/WPILib.h>
#include <frc/Solenoid.h>

using namespace frc;

class JackScrews {
  public:
  JackScrews(std::shared_ptr<Solenoid> frontAxleSolenoid, std::shared_ptr<Solenoid> rearAxleSolenoid) : 
    frontAxleSolenoid(frontAxleSolenoid), rearAxleSolenoid(rearAxleSolenoid) {}

  enum class LiftMode { kAll, kFront, kBack };

  void Run();

  void SetAllSolenoidState(bool extend);
  void SetFrontSolenoidState(bool extend);
  void SetRearSolenoidState(bool extend);

  // void SetExtendFL(bool extend);
  // void SetExtendFR(bool extend);
  // void SetExtendRL(bool extend);
  // void SetExtendRR(bool extend);

  void SetExtendScrews(bool extend, bool running);

  void SetLiftMode(LiftMode liftMode);
  void RunOpenLoop(double speed);

 private:
    std::shared_ptr<Solenoid> frontAxleSolenoid;
    std::shared_ptr<Solenoid> rearAxleSolenoid;

    bool running = false;
    bool direction = 1.0;

    LiftMode currentLiftMode = LiftMode::kAll;
    double openLoopSpeed = 0.0;
};
