/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/WPILib.h>
#include <frc/DoubleSolenoid.h>
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
  enum class ShiftMode { kDrive = true, kJackscrews = false };
  enum class LiftMode { kAll, kFront, kBack, kNone };
  enum class Direction { kNone = 0, kUp = -1, kDown = 1 };
  
  JackScrews();
  void Run() override;
  void Init() override;

  void ShiftAll(ShiftMode shiftMode);
  void ShiftFront(ShiftMode shiftMode);
  void ShiftRear(ShiftMode shiftMode);

  void SetLiftMode(LiftMode liftMode);
  void ConfigureOpenLoop(double speed, JackScrewControl::EndStateAction endStateAction = JackScrewControl::EndStateAction::kNone);
  void ConfigureControlled(LiftMode liftMode, Direction targetPosition, JackScrewControl::EndStateAction endStateAction);

  DriveInfo<std::shared_ptr<JackScrewControl>>* GetJackScrewControls() { return jackScrews.get(); }
  void SetMaxJackScrewSpeed(double speed_) { maxJackScrewSpeed = speed_; }

  bool IsEmergencyHalt() { return emergencyHalt; }

 private:
    std::unique_ptr<DriveInfo<std::shared_ptr<JackScrewControl>>> jackScrews;
    std::shared_ptr<DoubleSolenoid> frontAxleSolenoid;
    std::shared_ptr<DoubleSolenoid> rearAxleSolenoid;

    LiftMode currentLiftMode = LiftMode::kAll;
    Direction targetPosition = Direction::kNone;
    double controlTimeStart = -1;
    double maxJackScrewSpeed = 1.0;
    
    void DoControlled();
    DriveInfo<bool> DetermineJackScrewsToInit();
    bool CheckCANCommunications();

    const double kMaximumDisplacementThreshold = 10;   // threshold before adjusting speeds
    const double kHaltClimbDisplacementThreshold = 20;
    bool emergencyHalt = false;


};
