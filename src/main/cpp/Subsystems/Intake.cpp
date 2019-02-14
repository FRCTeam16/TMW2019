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
    beaterTop = RobotMap::beaterTopMotor;
    beaterBottom = RobotMap::beaterBottomMotor;
    ejectorSolenoid = RobotMap::ejectorSolenoid;
    hatchCatchSolenoid = RobotMap::hatchCatchSolenoid;
    gripperSolenoid = RobotMap::gripperSolenoid;
}

void Intake::Init() {
    bottomBeaterSpeed = 0.0;
    topBeaterSpeed = 0.0;
    ejectorSolenoidState = false;
    hatchSolenoidState = false;
    gripperSolenoidState = false;
    
    PrefUtil::getSet("Intake.IntakeCargo.bottomSpeed", 1.0);
    PrefUtil::getSet("Intake.IntakeCargo.topSpeed", 1.0);
    PrefUtil::getSet("Intake.EjectCargo.bottomSpeed", -1.0);
    PrefUtil::getSet("Intake.EjectCargo.topSpeed", -1.0);
    PrefUtil::getSet("Intake.IntakeHatch.bottomSpeed", -1.0);
    PrefUtil::getSet("Intake.IntakeHatch.topSpeed", 0.0);
    PrefUtil::getSet("Intake.EjectHatch.bottomSpeed", 1.0);
    PrefUtil::getSet("Intake.EjectHatch.topSpeed", 0.0);
}

void Intake::Run() {
    double elapsed = 0.0;
    if (startTime > 0) {
        elapsed = frc::Timer::GetFPGATimestamp() - startTime;
    }
    switch(currentState) {

        case IntakeState::kNone:
            //bottomBeaterSpeed = 0.0;
            //topBeaterSpeed = 0.0;
            // positionSpeed = 0.0;
        break;

        case IntakeState::kIntakeHatch:
            std::cout << "IntakeState::Run() -> kIntakeHatch (elapsed = " << elapsed << ")\n";
            runningSequence = true;

            ejectorSolenoidState = true;
            hatchSolenoidState = false;
            gripperSolenoidState = false;
            if (elapsed > 0.25 && elapsed < 0.5) {
                ejectorSolenoidState = true;
                gripperSolenoidState = true;
                hatchSolenoidState = false;
            } else if (elapsed >= 0.5) {
                ejectorSolenoidState = false;
                gripperSolenoidState = true;
                hatchSolenoidState = false;
                runningSequence = false;
                currentState = IntakeState::kNone; // human will tigger next state
            }
            break;

        case IntakeState::kEjectHatch:
            std::cout << "IntakeState::Run() -> kEjectHatch (elapsed = " << elapsed << ") " << ejectorSolenoidState << " - ";
            runningSequence = true;

            ejectorSolenoidState = false;
            // gripperSolenoidState = true;
            hatchSolenoidState = false;
            if (elapsed > 0.25 && elapsed < 0.5) {
                ejectorSolenoidState = true;
                gripperSolenoidState = false;
                hatchSolenoidState = false;
            } else if (elapsed >= 0.5 && elapsed < 1.0) {
                ejectorSolenoidState = true;
                gripperSolenoidState = false;
                hatchSolenoidState = true;
            } else if (elapsed >= 1.0) {
                ejectorSolenoidState = true;
                hatchSolenoidState = false;
                gripperSolenoidState = false;
                runningSequence = false;
                currentState = IntakeState::kNone;
            }
            break;
    }

    

    beaterBottom->Set(bottomBeaterSpeed);
    beaterTop->Set(topBeaterSpeed);
    
    // Safety Check
    // if (ejectorSolenoidState && !gripperSolenoidState) {
    //     std::cout << " Intake::Run() SAFETY OVERRIDE = toggling gripper solenoid\n";
    //     gripperSolenoidState = true;
    // }

    std::cout << "SOLENOIDS: ejector = " << ejectorSolenoidState << " | "
              << "gripper = " << gripperSolenoidState << " | "
              << "arms = " << hatchSolenoidState << "\n";

    ejectorSolenoid->Set(ejectorSolenoidState);
    hatchCatchSolenoid->Set(hatchSolenoidState);
    gripperSolenoid->Set(gripperSolenoidState);
}

void Intake::IntakeCargo() {
    bottomBeaterSpeed = PrefUtil::getSet("Intake.IntakeCargo.bottomSpeed", 1.0);
    topBeaterSpeed = PrefUtil::getSet("Intake.IntakeCargo.topSpeed", 1.0);
}

void Intake::EjectCargo() {
    bottomBeaterSpeed = PrefUtil::getSet("Intake.EjectCargo.bottomSpeed", -1.0);
    topBeaterSpeed = PrefUtil::getSet("Intake.EjectCargo.topSpeed", -1.0);
}

void Intake::IntakeHatch() {
    SetState(IntakeState::kIntakeHatch);
}

void Intake::EjectHatch() {
    std::cout << "Intake::EjectHatch\n";
    SetState(IntakeState::kEjectHatch);
}

void Intake::HatchIntakeFromGround() {
    bottomBeaterSpeed = PrefUtil::getSet("Intake.IntakeHatch.bottomSpeed", -1.0);
    topBeaterSpeed = PrefUtil::getSet("Intake.IntakeHatch.topSpeed", 0.0);
}

void Intake::HatchBeaterEject() {
    bottomBeaterSpeed = PrefUtil::getSet("Intake.EjectHatch.bottomSpeed", -1.0);
    topBeaterSpeed = PrefUtil::getSet("Intake.EjectHatch.topSpeed", 0.0);
}

void Intake::Stop() {
    bottomBeaterSpeed = 0.0;
    topBeaterSpeed = 0.0;
}

void Intake::SetState(IntakeState state) {
    startTime = frc::Timer::GetFPGATimestamp();
    currentState = state;
}

// Testing Methods

void Intake::SetBottomBeaterSpeed(double speed) {
    bottomBeaterSpeed = speed;
}

void Intake::SetTopBeaterSpeed(double speed) {
    topBeaterSpeed = speed;
}


void Intake::Instrument() {}