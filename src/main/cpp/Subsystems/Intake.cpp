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

#define M_PI		3.14159265358979323846	/* pi */
#define TWO_PI 6.28318530718

Intake::Intake() {
    rotateLeft = RobotMap::rotateLeftMotor;
    rotateRight = RobotMap::rotateRightMotor;
    rotateRight->SetInverted(true);
    rotateRight->Follow(*rotateLeft.get());

    rotateLeft->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Absolute);
    std::vector<std::shared_ptr<ctre::phoenix::motorcontrol::can::BaseMotorController>> motors {rotateLeft, rotateRight};
    for (auto const& motor : motors) {
        motor->SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
        // motor->ConfigPeakOutputForward(1.0);
        // motor->ConfigPeakOutputReverse(-1.0);
    }
    // TODO: limit constraints?

    rotateLeft->Config_kP(0, PrefUtil::getSet("Intake.rotate.P", 0.001));
    rotateLeft->Config_kI(0, PrefUtil::getSet("Intake.rotate.I", 0.0));
    rotateLeft->Config_kD(0, PrefUtil::getSet("Intake.rotate.D", 0.0));
    rotateLeft->Config_kF(0, 0.0);

    beaterTop = RobotMap::beaterTopMotor;
    beaterBottom = RobotMap::beaterBottomMotor;
    ejectorSolenoid = RobotMap::ejectorSolenoid;
    hatchCatchSolenoid = RobotMap::hatchCatchSolenoid;
    gripperSolenoid = RobotMap::gripperSolenoid;
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
    PrefUtil::getSet("Intake.IntakeHatch.bottomSpeed", -1.0);
    PrefUtil::getSet("Intake.IntakeHatch.topSpeed", 0.0);
    PrefUtil::getSet("Intake.EjectHatch.bottomSpeed", 1.0);
    PrefUtil::getSet("Intake.EjectHatch.topSpeed", 0.0);

    positionLookup[IntakePosition::kStarting] = PrefUtil::getSetInt("Intake.Positition.starting", 100);
    positionLookup[IntakePosition::kCargoShot] = PrefUtil::getSetInt("Intake.Positition.cargoshot", 200);
    positionLookup[IntakePosition::kLevelOne] = PrefUtil::getSetInt("Intake.Positition.levelone", 300);
    positionLookup[IntakePosition::kFloor] = PrefUtil::getSetInt("Intake.Positition.floor", 400);
    // todo: guess initial position?

    const int base = PrefUtil::getSetInt("Intake.position.base", 0);
    const int currentPositionValue = rotateLeft->GetSelectedSensorPosition(0);

    if (currentPositionValue < base) {
        std::cout << "!!! Intake - current positionless than base, removing loop !!!\n";
        rotateOffset = -4096;
    }
    targetPositionValue = currentPositionValue;
    positionControl = false;
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

    const int base = PrefUtil::getSetInt("Intake.position.base", 0);
    const int feedForwardZeroPos = PrefUtil::getSetInt("Intake.position.ffzeropos", 600);
    const double feedForwardZero = PrefUtil::getSet("Intake.position.ffzero", 0.11);

    int currentPosition = rotateLeft->GetSelectedSensorPosition(0);
    if (currentPosition < base) {
        currentPosition += 4096;
    }
    double theta = ((currentPosition - (base + feedForwardZeroPos)) / 4096.0) * TWO_PI;
    double k = feedForwardZero * cos(theta);
    frc::SmartDashboard::PutNumber("Rotate Angle", (theta * 180) / M_PI);

    if (positionControl) {
        rotateLeft->Config_kP(0, PrefUtil::getSet("Intake.rotate.P", 0.001));
        rotateLeft->Config_kI(0, PrefUtil::getSet("Intake.rotate.I", 0.0));
        rotateLeft->Config_kD(0, PrefUtil::getSet("Intake.rotate.D", 0.0));
        rotateLeft->Config_kF(0, k);

        rotateLeft->Set(ctre::phoenix::motorcontrol::ControlMode::Position, targetPositionValue + base + rotateOffset);
    } else {
        rotateLeft->Set(positionSpeed);
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


void Intake::SetIntakePosition(IntakePosition position) {
    targetPosition = position;
    targetPositionValue = positionLookup[targetPosition];
    positionControl = true;
}

void Intake::SetState(IntakeState state) {
    startTime = frc::Timer::GetFPGATimestamp();
    currentState = state;
}

void Intake::Instrument() {
    frc::SmartDashboard::PutNumber("Intake Pos", rotateLeft->GetSelectedSensorPosition(0));
    frc::SmartDashboard::PutNumber("Intake Target", targetPositionValue);
    frc::SmartDashboard::PutNumber("RotateLeft Amps", rotateLeft->GetOutputCurrent());
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

void Intake::SetPositionSpeed(double speed, bool flipMode) {
    positionSpeed = speed;
    if (flipMode) {
        positionControl = false;
    }
}
