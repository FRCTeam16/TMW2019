/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/IntakeRotate.h"
#include "RobotMap.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Util/PrefUtil.h"

#define M_PI		3.14159265358979323846	/* pi */
#define TWO_PI 6.28318530718

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
        // motor->ConfigPeakOutputForward(1.0);
        // motor->ConfigPeakOutputReverse(-1.0);
    }
    // TODO: limit constraints?
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
    positionLookup[IntakePosition::kLevelOne]  = PrefUtil::getSetInt("Intake.Positition.levelone", 300);
    positionLookup[IntakePosition::kFloor]     = PrefUtil::getSetInt("Intake.Positition.floor", 400);

    const int base = PrefUtil::getSetInt("Intake.position.base", 0);
    const int currentPositionValue = rotateLeft->GetSelectedSensorPosition(0);

    /**
     * If our current position is less than our base, then we probably had a power cycle
     * so we are reading the "real" values instead of having a full rotation added.  We just
     * offset by taking off a rotation if this is the case.
     */
    // if (currentPositionValue < base) {
    //     std::cout << "!!! Intake - current positionless than base, removing loop !!!\n";
    //     rotateOffset = -kRotation;
    // }
    targetPositionValue = currentPositionValue;
    positionControl = false;
}

void IntakeRotate::Run() {
    std::cout << "IntakeRotate::Run()\n";
    const int base = PrefUtil::getSetInt("Intake.position.base", 0);
    const int feedForwardZeroPos = PrefUtil::getSetInt("Intake.position.ffzeropos", 600);   // zero position for k
    const double feedForwardZero = PrefUtil::getSet("Intake.position.ffzero", 0.11);        // ff for holding zero

    int currentPosition = rotateLeft->GetSelectedSensorPosition(0);
    // if (currentPosition < base) {
    //     currentPosition += kRotation;
    // }

    double theta = ((currentPosition - (base + feedForwardZeroPos)) / kRotation) * TWO_PI;
    std::cout << "curPos = " << currentPosition << " | "
              << "base = " << base << " | "
              << "ffZP = " << feedForwardZeroPos << " | "
              << "kRot = " << kRotation << " | "
              << "2PI = " << TWO_PI << "\n";
    // double k = feedForwardZero * cos(theta / 2);    // Account for 2:1 gearing
    double k = 0;
    rotateLeft->Config_kF(0, k);
    const double rotateAngle = (theta * 180.0) / M_PI;
    frc::SmartDashboard::PutNumber("Rotate Angle", rotateAngle);
    std::cout << "rotateAngle: " << rotateAngle << "\n";
    const double computedTargetValue = targetPositionValue + base + rotateOffset;
   
    if (positionControl) {
        rotateLeft->Set(ControlMode::MotionMagic, computedTargetValue);
    } else {
        rotateLeft->Set(ControlMode::PercentOutput, positionSpeed);
    }
}

void IntakeRotate::SetIntakePosition(IntakePosition position) {
    targetPosition = position;
    targetPositionValue = positionLookup[targetPosition];
    positionControl = true;
}

void IntakeRotate::SetPositionSpeed(double speed, bool flipMode) {
    positionSpeed = speed;
    if (flipMode) {
        positionControl = false;
    }
}

void IntakeRotate::Instrument() {
    frc::SmartDashboard::PutNumber("Intake Pos", rotateLeft->GetSelectedSensorPosition(0));
    frc::SmartDashboard::PutNumber("Intake Target", targetPositionValue);
    frc::SmartDashboard::PutNumber("RotateLeft Amps", rotateLeft->GetOutputCurrent());
}