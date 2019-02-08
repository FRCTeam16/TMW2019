/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/JackScrews.h"
#include "Robot.h"
#include "RobotMap.h"
#include "Subsystems/Drive/DriveBase.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "Util/RampUtil.h"
#include <iostream>
#include <algorithm>

JackScrews::JackScrews() : frontAxleSolenoid(RobotMap::frontAxleSolenoid), rearAxleSolenoid(RobotMap::rearAxleSolenoid)
{
  auto wheels = Robot::driveBase->GetWheels();
  auto di = new DriveInfo<std::shared_ptr<JackScrewControl>>();
  di->FL.reset(new JackScrewControl(wheels.FL));
  di->FR.reset(new JackScrewControl(wheels.FR));
  di->RL.reset(new JackScrewControl(wheels.RL));
  di->RR.reset(new JackScrewControl(wheels.RR));
  jackScrews.reset(di);
}

void JackScrews::Init() {
  this->ShiftAll(JackScrews::ShiftMode::kDrive);
}

void JackScrews::Run()
{
  bool allSwerve =
    (JackScrewControl::JackScrewState::kSwerve == jackScrews->FL->GetCurrentState()) &&
    (JackScrewControl::JackScrewState::kSwerve == jackScrews->FR->GetCurrentState()) &&
    (JackScrewControl::JackScrewState::kSwerve == jackScrews->RL->GetCurrentState()) &&
    (JackScrewControl::JackScrewState::kSwerve == jackScrews->RR->GetCurrentState());
  
  if (allSwerve) {
    // Should be controlled by driveBase, not jackscrew
    std::cout << "!!! All jack screw controls in swerve mode, aborting !!!\n";
    return;
  }

  if (targetPosition != Direction::kNone) {
    DoControlled();
  }

  jackScrews->FL->Run();
  jackScrews->FR->Run();
  jackScrews->RL->Run();
  jackScrews->RR->Run();
  
  // std::cout << "Jackscrews::Run <=\n\n";
}

void JackScrews::ShiftAll(ShiftMode shiftMode) {
  ShiftFront(shiftMode);
  ShiftRear(shiftMode);
}

void JackScrews::ShiftFront(ShiftMode shiftMode) {
  frontAxleSolenoid->Set(static_cast<bool>(shiftMode));
  if (ShiftMode::kJackscrews == shiftMode) {
    std::cout << "Shifted front to open\n";
    jackScrews->FL->InitOpenLoop(0.0, JackScrewControl::EndStateAction::kNone);
    jackScrews->FR->InitOpenLoop(0.0, JackScrewControl::EndStateAction::kNone);
  } else {
    std::cout << "Shifted front to swerve\n";
    jackScrews->FL->SetCurrentState(JackScrewControl::JackScrewState::kSwerve);
    jackScrews->FR->SetCurrentState(JackScrewControl::JackScrewState::kSwerve);
  }
}

void JackScrews::ShiftRear(ShiftMode shiftMode) {
  rearAxleSolenoid->Set(static_cast<bool>(shiftMode));
  if (ShiftMode::kJackscrews == shiftMode) {
    std::cout << "Shifted rear to open\n";
    jackScrews->RL->InitOpenLoop(0.0, JackScrewControl::EndStateAction::kNone);
    jackScrews->RR->InitOpenLoop(0.0, JackScrewControl::EndStateAction::kNone);
  } else {
    std::cout << "Shifted rear to swerve\n";
    jackScrews->RL->SetCurrentState(JackScrewControl::JackScrewState::kSwerve);
    jackScrews->RR->SetCurrentState(JackScrewControl::JackScrewState::kSwerve);
  }
}

void JackScrews::SetLiftMode(LiftMode liftMode) {
  currentLiftMode = liftMode;
}


void JackScrews::ConfigureOpenLoop(double speed, JackScrewControl::EndStateAction endStateAction) {
  targetPosition = Direction::kNone;

  // Initialize jackscrews we will be controlling
  DriveInfo<bool> toInit {false};
  if (LiftMode::kFront == currentLiftMode) {
    toInit.FL = true;
    toInit.FR = true;
  } else if (LiftMode::kBack == currentLiftMode) {
    toInit.RL = true;
    toInit.RR = true;
  } else {
    toInit.FL = true;
    toInit.FR = true;
    toInit.RL = true;
    toInit.RR = true;
  }

  if (toInit.FL) { jackScrews->FL->InitOpenLoop(speed, endStateAction); }
  if (toInit.FR) { jackScrews->FR->InitOpenLoop(speed, endStateAction); }
  if (toInit.RL) { jackScrews->RL->InitOpenLoop(speed, endStateAction); }
  if (toInit.RR) { jackScrews->RR->InitOpenLoop(speed, endStateAction); }
}

void JackScrews::ConfigureControlled(LiftMode liftMode, Direction targetPosition_, JackScrewControl::EndStateAction endStateAction) {
  this->SetLiftMode(liftMode);
  targetPosition = targetPosition_;
  controlTimeStart = frc::Timer::GetFPGATimestamp();
  SetMaxJackScrewSpeed(1.0);

  frc::Preferences *prefs = frc::Preferences::GetInstance();

  // Initialize jackscrews we will be controlling
  DriveInfo<bool> toInit {false};
  if (LiftMode::kFront == currentLiftMode) {
    toInit.FL = true;
    toInit.FR = true;
  } else if (LiftMode::kBack == currentLiftMode) {
    toInit.RL = true;
    toInit.RR = true;
  } else {
    toInit.FL = true;
    toInit.FR = true;
    toInit.RL = true;
    toInit.RR = true;
  }

  if (toInit.FL) { jackScrews->FL->ConfigureControlled(prefs->GetDouble("JackScrew.FL.dist"), controlTimeStart, endStateAction); }
  if (toInit.FR) { jackScrews->FR->ConfigureControlled(prefs->GetDouble("JackScrew.FR.dist"), controlTimeStart, endStateAction); }
  if (toInit.RL) { jackScrews->RL->ConfigureControlled(prefs->GetDouble("JackScrew.RL.dist"), controlTimeStart, endStateAction); }
  if (toInit.RR) { jackScrews->RR->ConfigureControlled(prefs->GetDouble("JackScrew.RR.dist"), controlTimeStart, endStateAction); }
}

/**
 * Called from Run()
 */
void JackScrews::DoControlled() {
  std::vector<std::shared_ptr<JackScrewControl>> activeCalcs;
  std::vector<std::shared_ptr<JackScrewControl>> inactiveCalcs;
  if (LiftMode::kFront == currentLiftMode) {
    activeCalcs.push_back(jackScrews->FL);
    activeCalcs.push_back(jackScrews->FR);
  } else if (LiftMode::kBack == currentLiftMode) {
    activeCalcs.push_back(jackScrews->RL);
    activeCalcs.push_back(jackScrews->RR);
  } else {
    activeCalcs.push_back(jackScrews->FL);
    activeCalcs.push_back(jackScrews->FR);
    activeCalcs.push_back(jackScrews->RL);
    activeCalcs.push_back(jackScrews->RR);
  }
  std::cout << "Active calcs size: " << activeCalcs.size() << "\n";
  // Debug
  frc::SmartDashboard::PutNumber("FL Target", jackScrews->FL->GetTargetDistance());
  frc::SmartDashboard::PutNumber("FR Target", jackScrews->FR->GetTargetDistance());
  frc::SmartDashboard::PutNumber("RL Target", jackScrews->RL->GetTargetDistance());
  frc::SmartDashboard::PutNumber("RR Target", jackScrews->RR->GetTargetDistance());


  // Capture accumulated positions
  bool inHoldPosition = false;
  auto lowest = activeCalcs[0].get();
  for (int i=1; i < activeCalcs.size(); i++) {
    if (activeCalcs[i]->GetAccumulatedPosition() < lowest->GetAccumulatedPosition()) {
      lowest = activeCalcs[i].get();
      inHoldPosition |= activeCalcs[i]->IsClosedLoop();
    }
  }
  if (inHoldPosition) {
    std::cout << "Detected active jack screw control in hold position\n";
    return;
  }
  std::cout << "Configured lowest\n";

  const double kMinAccumulatedPosition = lowest->GetAccumulatedPosition();
  const auto speedDir = static_cast<int>(targetPosition);
  if (fabs(lowest->GetControlSpeed()) != maxJackScrewSpeed) {
    lowest->SetControlSpeed(maxJackScrewSpeed * speedDir);
  }
  std::cout << "Set lowest control speed to: " << (maxJackScrewSpeed * speedDir) << "\n";


  // check wheels outside of threshold, slow them down
  const double kMaximumDisplacementThreshold = 1;
  bool exitOpenLoop = false;
  for (int i=0; i<activeCalcs.size(); i++) {
    auto currentCalc = activeCalcs[i].get();
    if (currentCalc->IsClosedLoop()) {
      std::cout << "Closed Loop detected, exiting loop\n";
      exitOpenLoop = true;
      break;
    }

    const double diff = fabs(currentCalc->GetAccumulatedPosition() - kMinAccumulatedPosition);
    if (diff >= kMaximumDisplacementThreshold) {
      if (currentCalc->GetLastChange() > lowest->GetLastChange()) {
        const double delta = speedDir * 0.02; // alternate do ratio?
        const double newSpeed = currentCalc->GetControlSpeed() - delta;
        currentCalc->SetControlSpeed(newSpeed);
      }
    }
    std::string label = "calc[" + std::to_string(i) + "] Speed";
    frc::SmartDashboard::PutNumber(label, currentCalc->GetControlSpeed());

    std::cout << "calc[" << i << "] speed = " << currentCalc->GetControlSpeed()
              << " | change = " << currentCalc->GetLastChange()
              << " | accum = " << currentCalc->GetAccumulatedPosition()
              << "\n";

    
  } // end active calc

  if (exitOpenLoop) {
    for (auto const& control : activeCalcs) {
      control->Hold();
    }
  }
}