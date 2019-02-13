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

        case IntakeState::kIntakeCargo:
            bottomBeaterSpeed = PrefUtil::getSet("Intake.IntakeCargo.bottomSpeed", 1.0);
            topBeaterSpeed = PrefUtil::getSet("Intake.IntakeCargo.topSpeed", 1.0);
            break;

        case IntakeState::kEjectCargo:
            bottomBeaterSpeed = PrefUtil::getSet("Intake.EjectCargo.bottomSpeed", -1.0);
            topBeaterSpeed = PrefUtil::getSet("Intake.EjectCargo.topSpeed", -1.0);
            break;

        case IntakeState::kIntakeHatch:
            // std::cout << "IntakeState::Run() -> kIntakeHatch (elapsed = " << elapsed << ")\n";
            runningSequence = true;
            ejectorSolenoidState = true;
            hatchSolenoidState = true;
            gripperSolenoidState = true;
            runningSequence = false;
            // currentState = IntakeState::kNone; // human will tigger next state
            break;

        case IntakeState::kIntakeHatch2:
            // std::cout << "IntakeState::Run() -> kIntakeHatch2 (elapsed = " << elapsed << ")\n";
            runningSequence = true;
            gripperSolenoidState = false;
            if (elapsed > 0.25) {
                ejectorSolenoidState = false;
                runningSequence = false;
                currentState = IntakeState::kNone;
            }
            break;

        case IntakeState::kEjectHatch:
            // std::cout << "IntakeState::Run() -> kEjectHatch (elapsed = " << elapsed << ") " << ejectorSolenoidState << " - ";
            runningSequence = true;
            hatchSolenoidState = false;
            ejectorSolenoidState = true;
            if (elapsed > 0.25 && elapsed < 0.5) {
                gripperSolenoidState = true;
            } else if (elapsed > 0.5 ) {
                hatchSolenoidState = true;
                runningSequence = false;
                currentState = IntakeState::kNone;
            }
            break;
        
        case IntakeState::kOpen:
            // std::cout << "in kOpen: ";
            bottomBeaterSpeed = PrefUtil::getSet("Intake.EjectHatch.bottomSpeed", -1.0);
            topBeaterSpeed = PrefUtil::getSet("Intake.EjectHatch.topSpeed", 0.0);
            break;
    }

    

    beaterBottom->Set(bottomBeaterSpeed);
    beaterTop->Set(topBeaterSpeed);
    // FIXME: handling solenoid testing in Robot::TeleopPeriodic
    // std::cout << "---- Ejector set = " << ejectorSolenoidState << "\n";
    ejectorSolenoid->Set(ejectorSolenoidState);
    hatchCatchSolenoid->Set(hatchSolenoidState);
    gripperSolenoid->Set(gripperSolenoidState);
}

void Intake::IntakeCargo() {
    SetState(IntakeState::kIntakeCargo);
}

void Intake::EjectCargo() {
    SetState(IntakeState::kEjectCargo);
}

void Intake::IntakeHatch() {
    if (IntakeState::kIntakeHatch == currentState) {
        std::cout << "Intake::InjectHatch\n";
        SetState(IntakeState::kIntakeHatch2);
    } else {
        std::cout << "Intake::InjectHatch\n";
        SetState(IntakeState::kIntakeHatch);
    }
}

void Intake::EjectHatch() {
    std::cout << "Intake::EjectHatch\n";
    SetState(IntakeState::kEjectHatch);
}

void Intake::Stop() {
    // SetState(IntakeState::kNone);
    bottomBeaterSpeed = 0.0;
    topBeaterSpeed = 0.0;
}

void Intake::SetState(IntakeState state) {
    startTime = frc::Timer::GetFPGATimestamp();
    currentState = state;
}

// Testing Methods

void Intake::SetBottomBeaterSpeed(double speed) {
    // currentState = IntakeState::kOpen;
    bottomBeaterSpeed = speed;
}

void Intake::SetTopBeaterSpeed(double speed) {
    // currentState = IntakeState::kOpen;
    topBeaterSpeed = speed;
}


void Intake::Instrument() {}