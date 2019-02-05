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
#include "Subsystems/JackScrewControl.h"
#include <vector>

using namespace frc;

/**
 * JackScrew Controller
 * 
 * 1.75 motor turns per rotation 
 * 42 rotations
 * 73.5 per revolution
 * 0.2" revolution
 * 19" ~ 6700
 * 
 * 2/1/2019
 *  restart lift
    exit first step when all locked
    When one enters threshold all quit
 */
class JackScrews : public SubsystemManager
{
public:
  enum class ShiftMode { kDrive = false, kJackscrews = true };
  enum class LiftMode { kAll, kFront, kBack };
  enum class Position { kNone = 0, kUp = -1, kDown = 1 };
  
  JackScrews();
  void Run() override;
  void Init() override;

  void ShiftAll(ShiftMode shiftMode);
  void ShiftFront(ShiftMode shiftMode);
  void ShiftRear(ShiftMode shiftMode);

  void SetLiftMode(LiftMode liftMode);
  void ConfigureOpenLoop(double speed);
  void ConfigureControlled(LiftMode liftMode, Position targetPosition);

  bool InPosition();

  DriveInfo<std::shared_ptr<JackScrewControl>>* GetJackScrewControls() { return jackScrews.get(); }

 private:
    std::shared_ptr<Solenoid> frontAxleSolenoid;
    std::shared_ptr<Solenoid> rearAxleSolenoid;
    std::vector<std::shared_ptr<SwerveWheel>> allWheels;
    std::vector<std::shared_ptr<SwerveWheel>> frontAxis;
    std::vector<std::shared_ptr<SwerveWheel>> rearAxis;

    bool enabled = false;  // true when we are shifted and performing jackscrew manipulation
    LiftMode currentLiftMode = LiftMode::kAll;
    Position targetPosition = Position::kNone;
    double openLoopSpeed = 0.0;
    double controlTimeStart = -1;

    DriveInfo<bool> enabledCalculators;
    std::unique_ptr<DriveInfo<std::shared_ptr<JackScrewControl>>> jackScrews;
    bool controlHoldMode = false;
    
    void DoOpenLoop();
    void DoControlled();

};
