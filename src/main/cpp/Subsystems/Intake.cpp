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
}

void Intake::Init() {
    bottomBeaterSpeed = 0.0;
    topBeaterSpeed = 0.0;

}

void Intake::Run() {
    beaterBottom->Set(bottomBeaterSpeed);
    beaterTop->Set(topBeaterSpeed);
}

void Intake::IntakeCargo() {

}

void Intake::EjectCargo() {

}

void Intake::IntakeHatch() {

}

void Intake::EjectHatch() {

}


void Intake::SetIntakePosition(IntakePosition position) {
    // lookup numeric value for position
    // set motor position to numeric value
    
}

// Testing Methods

void Intake::SetBottomBeaterSpeed(double speed) {
    bottomBeaterSpeed = speed;
}

void Intake::SetTopBeaterSpeed(double speed) {
    topBeaterSpeed = speed;
}



