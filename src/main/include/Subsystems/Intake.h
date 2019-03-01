/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "ctre/Phoenix.h"
#include <map>
#include <memory>
#include "SubsystemManager.h"
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>


class IntakeBumpState {
public:
  bool started = false;
  bool bumpedUp = false;
  // bool bumpedDown = false; last move

  void Reset() {
    started = false;
    bumpedUp = false;
  }
};

class Intake: SubsystemManager {

public:
  Intake();

  void Init() override;
  void Run() override;
  void Instrument() override;

  void IntakeCargo();
  void EjectCargo();

  void IntakeHatch();
  void EjectHatch();

  void HatchIntakeFromGround();
  void HatchBeaterEject();

  /**
   * Stops motors, resets state
   */
  void Stop();


  
  void SetPositionSpeed(double speed, bool flipMode);

  void SetEjectorState(bool state) { ejectorSolenoidState = state; }
  void ToggleEjectorState() { ejectorSolenoidState = !ejectorSolenoidState; }
  void SetHatchState(bool state) { hatchSolenoidState = state; }
  void SetGripperState(bool state) { gripperSolenoidState = state; }


private:
  std::shared_ptr<WPI_TalonSRX> beaterTop;
  std::shared_ptr<WPI_VictorSPX> beaterBottom;

  std::shared_ptr<frc::Solenoid> ejectorSolenoid;
  std::shared_ptr<frc::Solenoid> hatchCatchSolenoid;
  std::shared_ptr<frc::Solenoid> gripperSolenoid;

  double bottomBeaterSpeed = 0.0;
  double topBeaterSpeed = 0.0;
  bool ejectorSolenoidState = false;
  bool hatchSolenoidState = false;
  bool gripperSolenoidState = false;

  enum class IntakeState{kNone, kIntakeHatch, kEjectHatch};
  IntakeState currentState = IntakeState::kNone;

  double startTime = -1;  // state activity start time
  bool runningSequence = false; // whether we are in a timed sequence
  bool ejectNeedsArms = false;

  void SetState(IntakeState state);
  void SetBottomBeaterSpeed(double speed);
  void SetTopBeaterSpeed(double speed);

  IntakeBumpState intakeBumpState;
};


