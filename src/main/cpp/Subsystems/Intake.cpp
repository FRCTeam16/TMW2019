/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Intake.h"
#include "RobotMap.h"

Intake::Intake() {
    rotateLeft.reset(RobotMap::rotateLeftMotor.get());
    
    rotateRight.reset(RobotMap::rotateRightMotor.get());

    beaterTop.reset(RobotMap::beaterTopMotor.get());

    beaterBottom.reset(RobotMap::beaterBottomMotor.get());
    
    ejectorSolenoid.reset(RobotMap::ejectorSolenoid.get());

    hatchCatchSolenoid.reset(RobotMap::hatchCatchSolenoid.get());
}

void Intake::Init() {
    bottomBeaterSpeed = 0.0;
    topBeaterSpeed = 0.0;

}

void Intake::Run() {
    switch(currentState) {
        case IntakeState::kNone:
            break;
        case IntakeState::kIntakeCargo:
            bottomBeaterSpeed = 1.0;
            topBeaterSpeed = 1.0;
            break;
        case IntakeState::kEjectCargo:
            break;
        case IntakeState::kIntakeHatch:
            break;
        case IntakeState::kEjectHatch:
            break;
    }

    beaterBottom->Set(bottomBeaterSpeed);
    beaterTop->Set(topBeaterSpeed);
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


void Intake::SetIntakePosition(IntakePosition position) {
    // lookup numeric value for position
    // set motor position to numeric value
    
}

void Intake::SetState(IntakeState state) {
    if(currentState == IntakeState::kNone) {
    currentState = state;
    }
}

// Testing Methods

void Intake::SetBottomBeaterSpeed(double speed) {
    bottomBeaterSpeed = speed;
}

void Intake::SetTopBeaterSpeed(double speed) {
    topBeaterSpeed = speed;
}



