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
    rotateLeft = RobotMap::rotateLeftMotor;
    rotateLeft->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Absolute);
    rotateRight = RobotMap::rotateRightMotor;
    rotateRight->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, rotateLeft->GetDeviceID());

    std::vector<std::shared_ptr<ctre::phoenix::motorcontrol::can::BaseMotorController>> motors {rotateLeft, rotateRight};
    for (auto const& motor : motors) {
        motor->SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
        motor->ConfigPeakOutputForward(1.0);
        motor->ConfigPeakOutputReverse(-1.0);
    }
    // TODO: limit constraints?

    rotateLeft->Config_kP(0, PrefUtil::getSet("Intake.rotate.P", 0.125));
    rotateLeft->Config_kP(0, PrefUtil::getSet("Intake.rotate.I", 0.0));
    rotateLeft->Config_kP(0, PrefUtil::getSet("Intake.rotate.D", 0.0));


    beaterTop = RobotMap::beaterTopMotor;
    beaterBottom = RobotMap::beaterBottomMotor;
    ejectorSolenoid = RobotMap::ejectorSolenoid;
    hatchCatchSolenoid = RobotMap::hatchCatchSolenoid;
    gripperSolenoid = RobotMap::gripperSolenoid;

    positionLookup[IntakePosition::kStarting] = PrefUtil::getSetInt("Intake.Positition.starting", 100);
    positionLookup[IntakePosition::kCargoShot] = PrefUtil::getSetInt("Intake.Positition.cargoshot", 200);
    positionLookup[IntakePosition::kLevelOne] = PrefUtil::getSetInt("Intake.Positition.levelone", 300);
    positionLookup[IntakePosition::kFloor] = PrefUtil::getSetInt("Intake.Positition.floor", 400);
    
    targetPositionValue = positionLookup[IntakePosition::kStarting];
}

void Intake::Init() {
    bottomBeaterSpeed = 0.0;
    topBeaterSpeed = 0.0;
    positionSpeed = 0.0;
    ejectorSolenoidState = false;
    hatchSolenoidState = false;
    gripperSolenoidState = false;
    
    PrefUtil::getSet("Intake.IntakeCargo.bottomSpeed", 1.0);
    PrefUtil::getSet("Intake.IntakeCargo.topSpeed", 1.0);
    PrefUtil::getSet("Intake.EjectCargo.bottomSpeed", -1.0);
    PrefUtil::getSet("Intake.EjectCargo.topSpeed", -1.0);
    PrefUtil::getSet("Intake.IntakeHatch.bottomSpeed", 1.0);
    PrefUtil::getSet("Intake.IntakeHatch.topSpeed", 0.0);
    PrefUtil::getSet("Intake.EjectHatch.bottomSpeed", -1.0);
    PrefUtil::getSet("Intake.EjectHatch.topSpeed", 0.0);

    targetPositionValue = rotateLeft->GetSelectedSensorPosition(0);
}

void Intake::Run() {
    const double elapsed = frc::Timer::GetFPGATimestamp() - startTime;
    switch(currentState) {
        case IntakeState::kNone:
            bottomBeaterSpeed = 0.0;
            topBeaterSpeed = 0.0;
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
            if (IntakePosition::kLevelOne == targetPosition) {
                runningSequence = true;
                ejectorSolenoidState = true;
                if (elapsed > 0.25) {
                    gripperSolenoidState = true;
                } else if (elapsed > 0.5) {
                    ejectorSolenoidState = false;
                    runningSequence = false;
                    currentState = IntakeState::kNone;
                } 
            } else {
                bottomBeaterSpeed = PrefUtil::getSet("Intake.IntakeHatch.bottomSpeed", 1.0);
                topBeaterSpeed = PrefUtil::getSet("Intake.IntakeHatch.topSpeed", 0.0);
            }
            break;

        case IntakeState::kEjectHatch:
            runningSequence = true;
            bottomBeaterSpeed = PrefUtil::getSet("Intake.EjectHatch.bottomSpeed", -1.0);
            topBeaterSpeed = PrefUtil::getSet("Intake.EjectHatch.topSpeed", 0.0);
            hatchSolenoidState = true;
            if (elapsed > 0.25) {
                ejectorSolenoidState = true;
            } else if (elapsed > 0.5) {
                runningSequence = false;
                currentState = IntakeState::kNone; // TODO: internal switch?
            }
            break;
    }

    rotateLeft->Config_kP(0, PrefUtil::getSet("Intake.rotate.P", 0.125));
    rotateLeft->Config_kP(0, PrefUtil::getSet("Intake.rotate.I", 0.0));
    rotateLeft->Config_kP(0, PrefUtil::getSet("Intake.rotate.D", 0.0));


    rotateLeft->Set(positionSpeed); // TODO: replace with position control targetPosition
    // rotateLeft->Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, targetPosition);
    beaterBottom->Set(bottomBeaterSpeed);
    beaterTop->Set(topBeaterSpeed);
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
    SetState(IntakeState::kIntakeHatch);
}

void Intake::EjectHatch() {
    SetState(IntakeState::kEjectHatch);
}

void Intake::Stop() {
    SetState(IntakeState::kNone);
}


void Intake::SetIntakePosition(IntakePosition position) {
    targetPosition = position;
    targetPositionValue = positionLookup[targetPosition];
}

void Intake::SetState(IntakeState state) {
    startTime = frc::Timer::GetFPGATimestamp();
}

void Intake::Instrument() {
    frc::SmartDashboard::PutNumber("Intake Pos", rotateLeft->GetSelectedSensorPosition(0));
    frc::SmartDashboard::PutNumber("Intake Target", targetPositionValue);
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
