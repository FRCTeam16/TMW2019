/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "ctre/Phoenix.h"
#include <memory>
#include "SubsystemManager.h"
#include "frc/Solenoid.h"

class Intake: SubsystemManager {

public:
  enum class IntakePosition { kStarting, kcargoPickup, kFloor, kLevelOne };

  Intake();

  void Init() override;
  void Run() override;

  void IntakeCargo();
  void EjectCargo();

  void IntakeHatch();
  void EjectHatch();

  void SetIntakePosition(IntakePosition position);
  void SetIntakePositionOpenLoop(double speed);
  // testing
  void SetBottomBeaterSpeed(double speed);
  void SetTopBeaterSpeed(double speed);


private:
  std::shared_ptr<WPI_TalonSRX> rotateLeft;
  std::shared_ptr<WPI_VictorSPX> rotateRight;
  std::shared_ptr<WPI_VictorSPX> beaterTop;
  std::shared_ptr<WPI_VictorSPX> beaterBottom;

  std::shared_ptr<Solenoid> ejectorSolenoid;
  std::shared_ptr<Solenoid> hatchCatchSolenoid;

  double bottomBeaterSpeed = 0.0;

  double topBeaterSpeed = 0.0;

  enum class IntakeState{kNone,kIntakeCargo,kEjectCargo,kIntakeHatch,kEjectHatch};
  IntakeState currentState = IntakeState::kNone;

  void SetState(IntakeState state);
};


