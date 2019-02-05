/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Lift/ThirdStep.h"
#include <iostream>

ThirdStep::ThirdStep() {}

void ThirdStep::Execute() {
    if (IsFirstRun()) {
        std::cout << "Running third step\n";
    }
}


/**
 
 const double now = frc::Timer::GetFPGATimestamp();
    if (positionStartTime == -1) {
        positionStartTime = now;
    }
    const double delta = (now - positionStartTime);
    // Wait for real position from caller, we want to know we are in a good position
    // Begin driving tank
    if (delta > 0.25) {
        double flSpeed = 0.0;
        double frSpeed = 0.0;
        auto wheels = Robot::driveBase->GetWheels();
        double value = Robot::oi->getDriverLeft()->GetY();
        if (value > kThreshold) {
            flSpeed = value;
        }
        value = Robot::oi->getDriverRight()->GetY();
        if (value > kThreshold) {
            frSpeed = value;
        }

        Robot::driveBase->SetTargetAngle(0.0);
        double twistInput = Robot::driveBase->GetCrabTwistOutput();
        wheels.FL->UseOpenLoopDrive(flSpeed);
        wheels.FR->UseOpenLoopDrive(frSpeed);
    }
*/
