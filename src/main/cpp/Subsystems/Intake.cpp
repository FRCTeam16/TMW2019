/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Intake.h"
#include "RobotMap.h"
#include "frc/Preferences.h"
#include "Util/PrefUtil.h"

Intake::Intake() {
    rotateLeft.reset(RobotMap::rotateLeftMotor.get());
    rotateLeft->SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    rotateRight.reset(RobotMap::rotateRightMotor.get());
    rotateRight->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, rotateLeft->GetDeviceID());
    rotateRight->SetNeutralMode(ctre::phoenix::motorcontrol::Brake);

    beaterTop.reset(RobotMap::beaterTopMotor.get());
    beaterBottom.reset(RobotMap::beaterBottomMotor.get());
    ejectorSolenoid.reset(RobotMap::ejectorSolenoid.get());
    hatchCatchSolenoid.reset(RobotMap::hatchCatchSolenoid.get());

    positionLookup[IntakePosition::kStarting] = PrefUtil::getSetInt("Intake.Positition.starting", 175);
    positionLookup[IntakePosition::kLevelOne] = PrefUtil::getSetInt("Intake.Positition.levelone", 250);
    positionLookup[IntakePosition::kCargoPickup] = PrefUtil::getSetInt("Intake.Positition.cargopickup", 350);
    positionLookup[IntakePosition::kFloor] = PrefUtil::getSetInt("Intake.Positition.floor", 500);
    targetPosition = positionLookup[IntakePosition::kStarting];
}

void Intake::Init() {
    bottomBeaterSpeed = 0.0;
    topBeaterSpeed = 0.0;
    positionSpeed = 0.0;
    ejectSolenoidState = false;
    hatchSolenoidState = false;
    

    PrefUtil::getSet("Intake.IntakeCargo.bottomSpeed", 1.0);
    PrefUtil::getSet("Intake.IntakeCargo.topSpeed", 1.0);
    PrefUtil::getSet("Intake.EjectCargo.bottomSpeed", -1.0);
    PrefUtil::getSet("Intake.EjectCargo.topSpeed", -1.0);
    PrefUtil::getSet("Intake.IntakeHatch.bottomSpeed", 1.0);
    PrefUtil::getSet("Intake.IntakeHatch.topSpeed", 0.0);
    PrefUtil::getSet("Intake.EjectHatch.bottomSpeed", -1.0);
    PrefUtil::getSet("Intake.EjectHatch.topSpeed", 0.0);
}

void Intake::Run() {
    const double elapsed = frc::Timer::GetFPGATimestamp() - startTime;
    switch(currentState) {
        case IntakeState::kNone:
            bottomBeaterSpeed = 0.0;
            topBeaterSpeed = 0.0;
            positionSpeed = 0.0;
            ejectSolenoidState = false;
            hatchSolenoidState = false;
            break;
        case IntakeState::kIntakeCargo:
            bottomBeaterSpeed = PrefUtil::getSet("Intake.IntakeCargo.bottomSpeed", 1.0);
            topBeaterSpeed = PrefUtil::getSet("Intake.IntakeCargo.topSpeed", 1.0);
            break;
        case IntakeState::kEjectCargo:
            bottomBeaterSpeed = PrefUtil::getSet("Intake.EjectCargo.bottomSpeed", -1.0);
            topBeaterSpeed = PrefUtil::getSet("Intake.EjectCargo.topSpeed", -1.0);
            break;
        case IntakeState::kIntakeHatch:
            bottomBeaterSpeed = PrefUtil::getSet("Intake.IntakeHatch.bottomSpeed", 1.0);
            topBeaterSpeed = PrefUtil::getSet("Intake.IntakeHatch.topSpeed", 0.0);
            break;
        case IntakeState::kEjectHatch:
            bottomBeaterSpeed = PrefUtil::getSet("Intake.EjectHatch.bottomSpeed", -1.0);
            topBeaterSpeed = PrefUtil::getSet("Intake.EjectHatch.topSpeed", 0.0);
            hatchSolenoidState = true;
            if (elapsed > 0.25) {
                ejectSolenoidState = true;
            } else if (elapsed > 0.5) {
                currentState = IntakeState::kNone; // TODO: internal switch?
            }
            break;
    }

    rotateLeft->Set(positionSpeed); // TODO: replace with position control targetPosition
    beaterBottom->Set(bottomBeaterSpeed);
    beaterTop->Set(topBeaterSpeed);
    ejectorSolenoid->Set(ejectSolenoidState);
    hatchCatchSolenoid->Set(hatchSolenoidState);
}

void Intake::IntakeCargo() {
    SetState(IntakeState::kIntakeCargo);
}

void Intake::EjectCargo() {
    SetState(IntakeState::kEjectCargo);
}

void Intake::IntakeHatch() {
    SetState(IntakeState::kIntakeHatch);
}

void Intake::EjectHatch() {
    SetState(IntakeState::kEjectHatch);
}

void Intake::Stop() {
    SetState(IntakeState::kNone);
}


void Intake::SetIntakePosition(IntakePosition position) {
    targetPosition = positionLookup[position];
}

void Intake::SetState(IntakeState state) {
    if(currentState == IntakeState::kNone) {
        startTime = frc::Timer::GetFPGATimestamp();
        currentState = state;
    } else {
        std::cout << "Ignoring request to switch state while active\n";
    }
}

void Intake::Instrument() {
    frc::SmartDashboard::PutNumber("Intake Pos", rotateLeft->GetSelectedSensorPosition(0));
    frc::SmartDashboard::PutNumber("Intake Target", targetPosition);
}

// Testing Methods

void Intake::SetBottomBeaterSpeed(double speed) {
    currentState = IntakeState::kOpen;
    bottomBeaterSpeed = speed;
}

void Intake::SetTopBeaterSpeed(double speed) {
    currentState = IntakeState::kOpen;
    topBeaterSpeed = speed;
}

void Intake::SetPositionSpeed(double speed) {
    positionSpeed = 0.0;
}
