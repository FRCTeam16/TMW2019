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
#include "frc/Solenoid.h"

class Intake: SubsystemManager {

public:
  enum class IntakePosition { kStarting, kCargoShot, kLevelOne, kFloor };

  Intake();

  void Init() override;
  void Run() override;
  void Instrument() override;

  void IntakeCargo();
  void EjectCargo();

  void IntakeHatch();
  void EjectHatch();

  /**
   * Stops motors, resets state
   */
  void Stop();

  void SetIntakePosition(IntakePosition position);

  // testing
  void SetBottomBeaterSpeed(double speed);
  void SetTopBeaterSpeed(double speed);
  void SetPositionSpeed(double speed);


private:
  std::shared_ptr<WPI_TalonSRX> rotateLeft;
  std::shared_ptr<WPI_VictorSPX> rotateRight;
  std::shared_ptr<WPI_VictorSPX> beaterTop;
  std::shared_ptr<WPI_VictorSPX> beaterBottom;

  std::shared_ptr<frc::Solenoid> ejectorSolenoid;
  std::shared_ptr<frc::Solenoid> hatchCatchSolenoid;
  std::shared_ptr<frc::Solenoid> gripperSolenoid;

  double bottomBeaterSpeed = 0.0;
  double topBeaterSpeed = 0.0;
  bool ejectorSolenoidState = false;
  bool hatchSolenoidState = false;
  bool gripperSolenoidState = false;

  enum class IntakeState{kNone, kOpen, kIntakeCargo, kEjectCargo, kIntakeHatch, kEjectHatch};
  IntakeState currentState = IntakeState::kNone;
  double startTime = -1;  // state activity start time
  bool runningSequence = false; // whether we are in a timed sequence

  std::map<Intake::IntakePosition, int> positionLookup;
  IntakePosition targetPosition = IntakePosition::kStarting;
  int targetPositionValue = 0;
  double positionSpeed = 0.0; // testing

  void SetState(IntakeState state);
};


