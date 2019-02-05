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
  frontAxis.push_back(wheels.FL);
  frontAxis.push_back(wheels.FR);
  rearAxis.push_back(wheels.RL);
  rearAxis.push_back(wheels.RR);

  allWheels.reserve(frontAxis.size() + rearAxis.size());
  allWheels.insert(allWheels.end(), frontAxis.begin(), frontAxis.end());
  allWheels.insert(allWheels.end(), rearAxis.begin(), rearAxis.end());

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
  if (!enabled) { return; }

  // std::cout << "Jackscrews::Run() =>\n";
  
  switch (targetPosition) {
    case Position::kNone:
      DoOpenLoop();
      break;

    case Position::kDown:
    case Position::kUp:
      DoControlled();      
      break;
  }
  
  // std::cout << "Jackscrews::Run <=\n\n";
}

void JackScrews::ShiftAll(ShiftMode shiftMode) {
  ShiftFront(shiftMode);
  ShiftRear(shiftMode);
  enabled = static_cast<bool>(shiftMode);
}

void JackScrews::ShiftFront(ShiftMode shiftMode) {
  frontAxleSolenoid->Set(static_cast<bool>(shiftMode));
  JackScrewControl::JackScrewState newState = JackScrewControl::JackScrewState::kSwerve;
  if (ShiftMode::kJackscrews == shiftMode) {
    newState = JackScrewControl::JackScrewState::kOpenLoop;
  }
  jackScrews->FL->SetCurrentState(newState);
  jackScrews->FR->SetCurrentState(newState);
}

void JackScrews::ShiftRear(ShiftMode shiftMode) {
  rearAxleSolenoid->Set(static_cast<bool>(shiftMode));
   JackScrewControl::JackScrewState newState = JackScrewControl::JackScrewState::kSwerve;
  if (ShiftMode::kJackscrews == shiftMode) {
    newState = JackScrewControl::JackScrewState::kOpenLoop;
  }
  jackScrews->RL->SetCurrentState(newState);
  jackScrews->RR->SetCurrentState(newState);
}

void JackScrews::SetLiftMode(LiftMode liftMode) {
  currentLiftMode = liftMode;
}

bool JackScrews::InPosition() {
  if (targetPosition != Position::kNone) {
    return controlHoldMode; // TODO: Check screw calculators?
  }
}

void JackScrews::ConfigureOpenLoop(double speed) {
  targetPosition = Position::kNone;
  openLoopSpeed = speed;
}

void JackScrews::ConfigureControlled(LiftMode liftMode, Position targetPosition_) {
  this->SetLiftMode(liftMode);
  targetPosition = targetPosition_;
  controlTimeStart = frc::Timer::GetFPGATimestamp();

  frc::Preferences *prefs = frc::Preferences::GetInstance();
  auto wheels = Robot::driveBase->GetWheels();

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

  if (toInit.FL) {
    jackScrews->FL->Init(prefs->GetDouble("JackScrew.FL.dist"), controlTimeStart);  
  }
  if (toInit.FR) {
    jackScrews->FR->Init(prefs->GetDouble("JackScrew.FR.dist"), controlTimeStart);
  }
  if (toInit.RL) {
    jackScrews->RL->Init(prefs->GetDouble("JackScrew.RL.dist"), controlTimeStart);
  }
  if (toInit.RR) {
    jackScrews->RR->Init(prefs->GetDouble("JackScrew.RR.dist"), controlTimeStart);
  }
}

/**
 * Called from Run()
 */
void JackScrews::DoOpenLoop() {
  // Determine the set of wheels we are manipulating
  std::vector<std::shared_ptr<SwerveWheel>> wheels;
  switch(currentLiftMode) {
    case LiftMode::kFront:
      wheels = frontAxis;
      break;
    case LiftMode::kBack:
      wheels = rearAxis;
      break;
    default:
      wheels = allWheels;
  }

  for (auto const &wheel : wheels) {
    wheel->UseOpenLoopDrive(openLoopSpeed);
  }
}

/**
 * Called from Run()
 */
void JackScrews::DoControlled() {
  std::vector<std::shared_ptr<JackScrewControl>> activeCalcs;
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

  if (controlHoldMode) {
    std::cout << "JackScrews::DoControlled Holding Position\n";
    for (auto const &calc : activeCalcs) {
      calc->Hold();
      // Debug
      frc::SmartDashboard::PutNumber("FL Target", jackScrews->FL->GetTargetDistance());
      frc::SmartDashboard::PutNumber("FR Target", jackScrews->FR->GetTargetDistance());
      frc::SmartDashboard::PutNumber("RL Target", jackScrews->RL->GetTargetDistance());
      frc::SmartDashboard::PutNumber("RR Target", jackScrews->RR->GetTargetDistance());
    }
    return;
  }
  
  // Capture accumulated positions
  auto lowest = activeCalcs[0].get();
  for (int i=1; i < activeCalcs.size(); i++) {
    if (activeCalcs[i]->GetAccumulatedPosition() < lowest->GetAccumulatedPosition()) {
      lowest = activeCalcs[i].get();
    }
  }
  std::cout << "Configured lowest\n";

  const double MAX_SPEED = 1.0;
  const double kMinAccumulatedPosition = lowest->GetAccumulatedPosition();
  const int speedDir = static_cast<int>(targetPosition);
  if (fabs(lowest->GetControlSpeed()) != MAX_SPEED) {
    lowest->SetControlSpeed(MAX_SPEED * speedDir);
  }
  std::cout << "Set lowest control speed to: " << (MAX_SPEED * speedDir) << "\n";


  // check wheels outside of threshold, slow them down
  const double kMaximumDisplacementThreshold = 1;
  bool exitOpenLoop = false;
  for (int i=0; i<activeCalcs.size(); i++) {
    // TODO: Check calc not in position control already? currently handled in calc
    auto currentCalc = activeCalcs[i].get();
    const int diff = abs(currentCalc->GetAccumulatedPosition() - kMinAccumulatedPosition);

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

    currentCalc->Run();
    if (currentCalc->IsClosedLoop()) {
      std::cout << "Closed Loop detected, exiting loop\n";
      exitOpenLoop = true;
      break;
    }
  } // end active calc

  if (exitOpenLoop) {
    for (auto const& control : activeCalcs) {
      control->Hold();
    }
  }
}