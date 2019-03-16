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
  di->FL.reset(new JackScrewControl("FL", wheels.FL));
  di->FR.reset(new JackScrewControl("FR", wheels.FR));
  di->RL.reset(new JackScrewControl("RL", wheels.RL));
  di->RR.reset(new JackScrewControl("RR", wheels.RR));
  jackScrews.reset(di);
}

void JackScrews::Init() {
  this->ShiftAll(JackScrews::ShiftMode::kDrive);

  kMaximumDisplacementThreshold = PrefUtil::getSet("JackScrew.speedDisplacementThreshold", 2.0);
  kHaltClimbDisplacementThreshold = PrefUtil::getSet("JackScrew.haltDisplacementThreshold", 2.0);
  speedDeltaDown = PrefUtil::getSet("JackScrew.deltaDown", 0.05);
  speedDeltaUp = PrefUtil::getSet("JackScrew.deltaUp", 0.05);
  enableEmergencyHalt = PrefUtil::getSetBool("JackScrew.EnableEStop", true);

  jackScrews->FL->Init();
  jackScrews->FR->Init();
  jackScrews->RL->Init();
  jackScrews->RR->Init();
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
    // std::cout << "!!! All jack screw controls in swerve mode, aborting !!!\n";
    return;
  }

  // Check CAN communications
  emergencyHalt |= CheckCANCommunications();
  if (emergencyHalt) {
    std::cout << "!!! JackScrews climbing disabled due to emergency halt !!!\n";
    auto wheels = Robot::driveBase->GetWheels();
    wheels.FL->UseOpenLoopDrive();
    wheels.FR->UseOpenLoopDrive();
    wheels.RL->UseOpenLoopDrive();
    wheels.RR->UseOpenLoopDrive();
    return;
  }


  if (targetPosition != Direction::kNone) {
    // DoControlled();
    ControlRamp();
  }

  std::cout << "\nFL | ";
  jackScrews->FL->Run();
  std::cout << "\nFR | ";
  jackScrews->FR->Run();
  std::cout << "\nRL | ";
  jackScrews->RL->Run();
  std::cout << "\nRR | ";
  jackScrews->RR->Run();
  std::cout << "\n";
  // std::cout << "Jackscrews::Run <=\n\n";
}

bool JackScrews::CheckCANCommunications() {
  bool halt = false;
  auto wheels = Robot::driveBase->GetWheels();
  if (wheels.FL->HasCANError()) {
    std::cout << "!!! CAN Error FL, emergency halt !!!\n";
    halt = true;
  }
  if (wheels.FR->HasCANError()) {
    std::cout << "!!! CAN Error FR, emergency halt !!!\n";
    halt = true;
  }
  if (wheels.RL->HasCANError()) {
    std::cout << "!!! CAN Error RL, emergency halt !!!\n";
    halt = true;
  }
  if (wheels.RR->HasCANError()) {
    std::cout << "!!! CAN Error RR, emergency halt !!!\n";
    halt = true;
  }
  return halt;
}

void JackScrews::ShiftAll(ShiftMode shiftMode) {
  ShiftFront(shiftMode);
  ShiftRear(shiftMode);
}

void JackScrews::ShiftFront(ShiftMode shiftMode) {
  const bool shiftBool = static_cast<bool>(shiftMode);
  frontAxleSolenoid->Set(shiftBool ? DoubleSolenoid::Value::kForward : DoubleSolenoid::Value::kReverse);
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
  const bool shiftBool = static_cast<bool>(shiftMode);
  rearAxleSolenoid->Set(shiftBool ? DoubleSolenoid::Value::kForward : DoubleSolenoid::Value::kReverse);
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
  DriveInfo<bool> toInit = DetermineJackScrewsToInit();

  if (toInit.FL) { jackScrews->FL->InitOpenLoop(speed, endStateAction); }
  if (toInit.FR) { jackScrews->FR->InitOpenLoop(speed, endStateAction); }
  if (toInit.RL) { jackScrews->RL->InitOpenLoop(speed, endStateAction); }
  if (toInit.RR) { jackScrews->RR->InitOpenLoop(speed, endStateAction); }
}

void JackScrews::ConfigureControlled(LiftMode liftMode, Direction targetPosition_, JackScrewControl::EndStateAction endStateAction, bool doRamp) {
  currentLiftMode = liftMode;
  this->SetLiftMode(liftMode);
  targetPosition = targetPosition_;
  controlTimeStart = frc::Timer::GetFPGATimestamp();
  SetMaxJackScrewSpeed(1.0);  // reset

  frc::Preferences *prefs = frc::Preferences::GetInstance();

  // Initialize jackscrews we will be controlling
  DriveInfo<bool> toInit = DetermineJackScrewsToInit();
  
  int dirMul = Direction::kUp == targetPosition ? -1 : 1;
  const double controlSpeed = maxJackScrewSpeed * dirMul;
  const double distance = prefs->GetDouble("JackScrew.dist") * dirMul;
  if (toInit.FL) { jackScrews->FL->ConfigureControlled(distance, controlSpeed, controlTimeStart, endStateAction, doRamp); }
  if (toInit.FR) { jackScrews->FR->ConfigureControlled(distance, controlSpeed, controlTimeStart, endStateAction, doRamp); }
  if (toInit.RL) { jackScrews->RL->ConfigureControlled(distance, controlSpeed, controlTimeStart, endStateAction, doRamp); }
  if (toInit.RR) { jackScrews->RR->ConfigureControlled(distance, controlSpeed, controlTimeStart, endStateAction, doRamp); }
}

DriveInfo<bool> JackScrews::DetermineJackScrewsToInit() {
  DriveInfo<bool> toInit {false};
  switch (currentLiftMode) {
    case LiftMode::kFront:
      toInit.FL = true;
      toInit.FR = true;
      break;
    case LiftMode::kBack:
      toInit.RL = true;
      toInit.RR = true;
      break;
    case LiftMode::kAll:
      toInit.FL = true;
      toInit.FR = true;
      toInit.RL = true;
      toInit.RR = true;
      break;
    case LiftMode::kNone:
      // handled by initializtion
      break;
  }
  return toInit;
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
  // std::cout << "Active calcs size: " << activeCalcs.size() << "\n";


  // Capture accumulated positions
  // Calculate lowest jackscrew based on accumulation
  bool inHoldPosition = false;
  auto lowest = activeCalcs[0].get();
  std::vector<double> positions(activeCalcs.size());
  positions[0] = lowest->GetAccumulatedPosition();
  for (int i=1; i < activeCalcs.size(); i++) {
    const double accumulated = activeCalcs[i]->GetAccumulatedPosition(); 
    positions[i] = accumulated;
    if (fabs(accumulated)  < fabs(lowest->GetAccumulatedPosition())) {
      lowest = activeCalcs[i].get();
      std::cout << "lowest = " << i << " = " << accumulated << "\n";
      inHoldPosition |= activeCalcs[i]->IsClosedLoop();
    }
  }


  // If any of the stored accumulated positions exceed the halt threshold then we
  // stop climbing
  for (int i=0; i<activeCalcs.size(); i++) {
    const double diff = fabs(positions[i] - lowest->GetAccumulatedPosition());
    if (diff > kHaltClimbDisplacementThreshold) {
      std::cout << "****** Exceeded halt climb displacement threshold at position " << i
                << ", displacement was " << diff << "*****\n";
      std::cout << "Position[" << i << "] = " << positions[i] << " | lowest = " << lowest->GetAccumulatedPosition() << "\n";
      emergencyHalt = enableEmergencyHalt ? true : false;
    }
  }

  // Emergency Halt Handler
  if (emergencyHalt) {
    jackScrews->FL->InitOpenLoop(0.0, JackScrewControl::EndStateAction::kNone);
    jackScrews->FR->InitOpenLoop(0.0, JackScrewControl::EndStateAction::kNone);
    jackScrews->RL->InitOpenLoop(0.0, JackScrewControl::EndStateAction::kNone);
    jackScrews->RR->InitOpenLoop(0.0, JackScrewControl::EndStateAction::kNone);
    auto wheels = Robot::driveBase->GetWheels();
    // TOOD: Indicate error to callers?
    this->SetLiftMode(JackScrews::LiftMode::kAll);
    this->ConfigureOpenLoop(0.0, JackScrewControl::EndStateAction::kNone);
    return;
  }

  
  // If any of the active calcs have entered close loop control, then we stop
  // processing here
  if (inHoldPosition) {
    std::cout << "Detected active jack screw control in hold position\n";
    return;
  }

  const double lowestPosition = lowest->GetAccumulatedPosition();
  const auto speedDir = static_cast<int>(targetPosition);
  if (fabs(lowest->GetControlSpeed()) != maxJackScrewSpeed) {
    lowest->SetControlSpeed(maxJackScrewSpeed * speedDir);
  }
  std::cout << "Set lowest control speed to: " << (maxJackScrewSpeed * speedDir) << "\n";


  // Calculate what our maximum displacement threshold should be
  if (lowestPosition < 5) {
    kMaximumDisplacementThreshold = 0.5;
  } else if (lowestPosition < 10) {
    kMaximumDisplacementThreshold = 1;
  } else if (lowestPosition < 20) {
    kMaximumDisplacementThreshold = 1.5;
  } else {
    kMaximumDisplacementThreshold = 2.0;
  }
  std::cout << "Set kMaximumDisplacementThreshold to " << kMaximumDisplacementThreshold << "\n";

  // check wheels outside of threshold, slow them down
  bool exitOpenLoop = false;
  for (int i=0; i<activeCalcs.size(); i++) {
    auto currentCalc = activeCalcs[i].get();
    
    if (currentCalc->IsClosedLoop()) {
      std::cout << "Closed Loop detected, exiting loop\n";
      exitOpenLoop = true;
      break;
    }

    const double diff = fabs(currentCalc->GetAccumulatedPosition() - lowestPosition);
    if (diff >= kMaximumDisplacementThreshold) {
      if (fabs(currentCalc->GetLastChange()) > fabs(lowest->GetLastChange())) {
        const double baseDelta = (targetPosition == Direction::kDown) ? speedDeltaDown : speedDeltaUp;
        const double delta = speedDir * baseDelta; // alternate do ratio? // FIXME
        double newSpeed = currentCalc->GetControlSpeed() - delta;

        // Make sure we never go below zero
        if ((targetPosition == Direction::kDown && newSpeed < 0.0) || (targetPosition == Direction::kUp && newSpeed > 0.0)) {
          std::cout << "!!! Safety stop to 0.0 because adjusted speed rolled over zero\n";
          newSpeed = 0.0;
        }
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


inline void logJSC(int id, JackScrewControl *calc) {
  std::cout << "calc[" << id << "] speed = " << calc->GetControlSpeed()
          << " | change = " << calc->GetLastChange()
          << " | accum = " << calc->GetAccumulatedPosition()
          << " | closed = " << calc->IsClosedLoop()
          << "\n";
}


inline double SafetyCheckSpeed(double speed, JackScrews::Direction targetPosition) {
  if (targetPosition == JackScrews::Direction::kDown) {
    if (speed < 0) {
      std::cout << "!!! SPEED NEGATIVE DURING DOWN DRIVE !!!\n";
      return 0.0;
    }
  } else if (targetPosition == JackScrews::Direction::kUp) {
    if (speed > 0) {
      std::cout << "!!! SPEED POSITIVE DURING UP DRIVE !!!\n";
      return 0.0;
    }
  }
  return speed;
}

void JackScrews::ControlRamp() {

  double frontSpeed = PrefUtil::getSet("JackScrew.cr.frontSpeed", 1.0);
  double backSpeed = PrefUtil::getSet("JackScrew.cr.backSpeed", 0.85);
  double rampTime = PrefUtil::getSet("JackScrew.cr.rampTime", 0.5);

  const double spinUpTime = 0.25;
  const double spinUpSpeed = 0.10;
  const double now = frc::Timer::GetFPGATimestamp();
  double elapsed = now - controlTimeStart;
  if (elapsed < spinUpTime) {
    frontSpeed = spinUpSpeed;
    backSpeed = spinUpSpeed;
  } else {
    elapsed -= spinUpTime;
    frontSpeed = RampUtil::RampUp(frontSpeed, elapsed, rampTime, spinUpSpeed);
    backSpeed = RampUtil::RampUp(backSpeed, elapsed, rampTime, spinUpSpeed);
  }

  // Set direction we are running the motors based on target position direction (up/down)
  const auto speedDir = static_cast<int>(targetPosition);
  frontSpeed *= speedDir;
  backSpeed *= speedDir;

  // Safety Prevention on speed direction
  frontSpeed = SafetyCheckSpeed(frontSpeed, targetPosition);
  backSpeed = SafetyCheckSpeed(backSpeed, targetPosition);

  if (LiftMode::kFront == currentLiftMode || LiftMode::kAll == currentLiftMode) {
    jackScrews->FL->SetControlSpeed(frontSpeed);
    jackScrews->FR->SetControlSpeed(frontSpeed);

    logJSC(0, jackScrews->FL.get());
    logJSC(1, jackScrews->FR.get());

  } else if (LiftMode::kBack == currentLiftMode || LiftMode::kAll == currentLiftMode) {
    // We use front speed if we are in back only mode since it is fastest
    double adjustedBackSpeed = LiftMode::kBack == currentLiftMode ? frontSpeed : backSpeed;

    jackScrews->RL->SetControlSpeed(adjustedBackSpeed);
    jackScrews->RR->SetControlSpeed(adjustedBackSpeed);

    logJSC(2, jackScrews->RL.get());
    logJSC(3, jackScrews->RR.get());
  } 

}


void JackScrews::FullSpeedC() {
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

  // If any active screws have entered closed loop,
  // we'll exit and let the jack screw control handle
  // final positioning
  const auto speedDir = static_cast<int>(targetPosition);
  for (int i=0; i < activeCalcs.size(); i++) {
    auto currentCalc = activeCalcs[i].get();
    if (currentCalc->IsClosedLoop()) {
      std::cout << "Jack Screws in final position, exiting main control and having JSC handle\n";
      return;
    }
    // Set maximum speed
    const double screwSpeed = maxJackScrewSpeed * speedDir;
    currentCalc->SetControlSpeed(screwSpeed);
     std::cout << "calc[" << i << "] speed = " << currentCalc->GetControlSpeed()
              << " | change = " << currentCalc->GetLastChange()
              << " | accum = " << currentCalc->GetAccumulatedPosition()
              << "\n";
  } 
}