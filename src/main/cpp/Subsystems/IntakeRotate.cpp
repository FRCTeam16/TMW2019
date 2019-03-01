/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/IntakeRotate.h"
#include "Robot.h"
#include "RobotMap.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Util/PrefUtil.h"

#define M_PI		3.14159265358979323846	/* pi */
#define TWO_PI      6.28318530718

const int kRotation = 4096;

IntakeRotate::IntakeRotate() {
    rotateLeft = RobotMap::rotateLeftMotor;
    rotateRight = RobotMap::rotateRightMotor;
    rotateRight->SetInverted(true);
    rotateRight->Follow(*rotateLeft.get());

    rotateLeft->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Absolute);
    std::vector<std::shared_ptr<ctre::phoenix::motorcontrol::can::BaseMotorController>> motors {rotateLeft, rotateRight};
    for (auto const& motor : motors) {
        motor->SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    }
}

void IntakeRotate::Init() {
    rotateLeft->Config_kP(0, PrefUtil::getSet("Intake.rotate.P", 0.001));
    rotateLeft->Config_kI(0, PrefUtil::getSet("Intake.rotate.I", 0.0));
    rotateLeft->Config_kD(0, PrefUtil::getSet("Intake.rotate.D", 0.0));
    rotateLeft->Config_kF(0, 0);
    rotateLeft->ConfigMotionCruiseVelocity(PrefUtil::getSet("Intake.rotate.V", 500));
    rotateLeft->ConfigMotionAcceleration(PrefUtil::getSet("Intake.rotate.A", 500));


    positionLookup[IntakePosition::kStarting]  = PrefUtil::getSetInt("Intake.Positition.starting", 100);
    positionLookup[IntakePosition::kCargoShot] = PrefUtil::getSetInt("Intake.Positition.cargoshot", 200);
    positionLookup[IntakePosition::kRocketShot] = PrefUtil::getSetInt("Intake.Positition.rocketshot", 200);
    positionLookup[IntakePosition::kLevelOne]  = PrefUtil::getSetInt("Intake.Positition.levelone", 300);
    positionLookup[IntakePosition::kCargoPickup] = PrefUtil::getSetInt("Intake.Position.cargopickup", 350);
    positionLookup[IntakePosition::kFloor]     = PrefUtil::getSetInt("Intake.Positition.floor", 400);

    const int base = PrefUtil::getSetInt("Intake.position.base", 0);
    const int currentPositionValue = rotateLeft->GetSelectedSensorPosition(0);
    targetPositionValue = currentPositionValue - base;
    rotateLeft->ClearMotionProfileTrajectories();
    rotateLeft->ClearMotionProfileHasUnderrun();

    positionControl = false;
    initializeFinished = false;
    initializeScanCounts = 0;

    std::cout << "IntakeRotate::Init() curPos = " << currentPositionValue << " | "
            << "Target: " << targetPositionValue << " | "
            << "Computed Target: " << computedTargetValue << "\n";
}

void IntakeRotate::Run() {

    // std::cout << "IntakeRotate::Run(" << positionControl << ")\n";
    const double base = PrefUtil::getSetInt("Intake.position.base", 0);
    const int feedForwardZeroPos = PrefUtil::getSetInt("Intake.position.ffzeropos", 600);   // zero position for k
    const double feedForwardZero = PrefUtil::getSet("Intake.position.ffzero", 0.11);        // ff for holding zero

    const int currentPosition = rotateLeft->GetSelectedSensorPosition(0);
    const double theta = (currentPosition - (base + feedForwardZeroPos)) * (M_PI / kRotation);    // use M_PI instead of TWO_PI to account for 1:2 gearing
    rotateAngle = (theta * 180.0) / M_PI;
    // const double k = feedForwardZero * cos(theta);    // Account for 2:1 gearing
    const double k = 0;
             
    rotateLeft->Config_kF(0, k);
    frc::SmartDashboard::PutNumber("Intake Target (non-computed)", targetPositionValue);
    computedTargetValue = targetPositionValue + base;

    
    // std::cout << "IntakeRotate::Run() curPos = " << currentPosition << " | "
    //           << "Target: " << targetPositionValue << " | "
    //           << "Computed Target: " << computedTargetValue << " --- "
    //           << "base = " << base << " | "
    //           << "ffZP = " << feedForwardZeroPos << " = "
    //           << "raw theta (rad) = " << theta 
    //           << " theta (deg) = " << rotateAngle << " | "
    //           << "k = " << k << "\n";

    if (IntakeRotate::IntakePosition::kFloor == targetPosition ||
        IntakeRotate::IntakePosition::kCargoPickup == targetPosition) {
        Robot::intake->SetEjectorState(false);  // force retract of ejector if we move to floor
        Robot::intake->SetGripperState(false);  // Make sure gripper is retracted as well
    }

    bool doPositionControl = positionControl;
    if (!initializeFinished && (initializeScanCounts++ < kInitializeScanCountMax)) {
        // std::cout << "Overriding intitialization - current draw: " 
        //           << rotateLeft->GetOutputCurrent() << "\n";
        doPositionControl = false;
        positionSpeed = 0.0;
    } else {
        if (initializeScanCounts > 0) {
            initializeFinished = true;
            initializeScanCounts = 0;
            positionControl = true;
            targetPositionValue = currentPosition - base;
        }   
    }
   
    if (doPositionControl) {
        rotateLeft->Set(ControlMode::MotionMagic, computedTargetValue);
    } else {
        rotateLeft->Set(ControlMode::PercentOutput, positionSpeed);
    }
}

void IntakeRotate::SetIntakePosition(IntakePosition position) {
    std::cout << "IntakeRotate::SetIntakePosition\n";
    positionLookup[IntakePosition::kStarting]  = PrefUtil::getSetInt("Intake.Positition.starting", 100);
    positionLookup[IntakePosition::kCargoShot] = PrefUtil::getSetInt("Intake.Positition.cargoshot", 200);
    positionLookup[IntakePosition::kRocketShot] = PrefUtil::getSetInt("Intake.Positition.rocketshot", 200);
    positionLookup[IntakePosition::kLevelOne]  = PrefUtil::getSetInt("Intake.Positition.levelone", 300);
    positionLookup[IntakePosition::kCargoPickup] = PrefUtil::getSetInt("Intake.Position.cargopickup", 350);
    positionLookup[IntakePosition::kFloor]     = PrefUtil::getSetInt("Intake.Positition.floor", 400);

    targetPosition = position;
    targetPositionValue = positionLookup[targetPosition];
    positionControl = true;
    positionSpeed = 0.0;
}

 IntakeRotate::IntakePosition IntakeRotate::GetIntakePosition() {
     return targetPosition;
 }

 
void IntakeRotate::SetPositionSpeed(double speed, bool openLoop) {
    positionSpeed = speed;

    if (openLoop) {
        // std::cout << "Setting open loop from SetPositionSpeed\n";
        positionControl = false;
        positionSpeed = positionSpeed/3.5;
    } else if (!positionControl) {
        // std::cout << "Setting CLOSED loop from SetPositionSpeed\n";
        double currentPosition = rotateLeft->GetSelectedSensorPosition(0);
        const double base = PrefUtil::getSetInt("Intake.position.base", 0);
        targetPositionValue = currentPosition - base;
        positionControl = true;
        // std::cout << "SetPositionSpeed() -> Current Pos: " << currentPosition << " | "
        //           << "Base: " << base << " = "
        //           << targetPositionValue << "\n";
    }
}

void IntakeRotate::DisabledHoldCurrentPosition() {
    // double currentPosition = rotateLeft->GetSelectedSensorPosition(0);
    // const double base = PrefUtil::getSetInt("Intake.position.base", 0);
    // targetPositionValue = currentPosition - base;
    // positionControl = true;
    // rotateLeft->Set(ControlMode::MotionMagic, currentPosition);      // make sure to signal
    rotateLeft->Set(ControlMode::PercentOutput, 0.0);
}

void IntakeRotate::CalibrateHome() {
    const int currentPosition = rotateLeft->GetSelectedSensorPosition();
    frc::Preferences::GetInstance()->PutInt("Intake.position.base", currentPosition);
    std::cout << "*** CalibrateIntakeRotate: Base is now " << currentPosition << "\n";
}

void IntakeRotate::Instrument() {
    const int currentPosition = rotateLeft->GetSelectedSensorPosition();
    const double base = PrefUtil::getSetInt("Intake.position.base", 0);

    frc::SmartDashboard::PutNumber("Intake Pos", currentPosition);
    frc::SmartDashboard::PutNumber("Intake Target", computedTargetValue);
    frc::SmartDashboard::PutNumber("Virtual Pos", base + currentPosition);
    frc::SmartDashboard::PutNumber("RotateLeft Amps", rotateLeft->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Rotate Angle", rotateAngle);
}