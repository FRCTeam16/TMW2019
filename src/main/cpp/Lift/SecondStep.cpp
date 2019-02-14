/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Lift/SecondStep.h"
#include <iostream>
#include "Robot.h"
#include "Subsystems/JackScrews.h"


void SecondStep::Execute() {
    const double kThreshold = 0.1;
    auto jackScrewControls = Robot::jackScrews->GetJackScrewControls();

    if (IsFirstRun()) {
        std::cout << "Lift Second Step\n";
        Robot::jackScrews->ConfigureControlled(
                JackScrews::LiftMode::kFront,
                JackScrews::Direction::kUp,
                JackScrewControl::EndStateAction::kSwitchToAmpDetect);
    } else {
        if (!liftFinished) {
            // Waiting for front jackscrew amp detection to kickout
            bool leftFinished = jackScrewControls->FL->IsFinished();
            bool rightFinished = jackScrewControls->FR->IsFinished();
            liftFinished = leftFinished && rightFinished;
        } else {
            if (!shiftedFrontToSwerve) {
                Robot::jackScrews->ShiftFront(JackScrews::ShiftMode::kDrive);
                Robot::jackScrews->SetLiftMode(JackScrews::LiftMode::kNone);
                
                shiftedFrontToSwerve = true;
                Robot::driveBase->SetTargetAngle(-180.0);
            } else {
                Robot::driveBase->SetTargetAngle(-180.0);
                const double kLowJoyThreshold = 0.10;
                const double kHighJoyThreshold = 0.30;
                double leftInput = Robot::oi->getDriverLeft()->GetY();
                double rightInput = Robot::oi->getDriverRight()->GetY();
                if (fabs(leftInput) < kLowJoyThreshold) { leftInput = 0.0; }
                if (fabs(leftInput) > kHighJoyThreshold) { leftInput = kHighJoyThreshold; }
                if (fabs(rightInput) < kLowJoyThreshold) { rightInput = 0.0; }
                if (fabs(rightInput) > kHighJoyThreshold) { rightInput = kHighJoyThreshold; }

                std::cout << "Left: " << leftInput << " | Right: " << rightInput << "\n";
 
                if (Robot::oi->DL9->Pressed()) {
                    const double crabSpeed = PrefUtil::getSet("Lift.step2.drivespeed.y", -0.2);
                    liftDrive.DriveFront(Robot::driveBase->GetCrabTwistOutput(), crabSpeed, 0, true);
                } else {
                    liftDrive.DriveTank(leftInput, rightInput);
                }
                
                // JackScrewControl does not handle swerve inputs so we must send motor inputs
                auto jsCtrls = Robot::jackScrews->GetJackScrewControls();
                jsCtrls->RL->Run();
                jsCtrls->RR->Run();
                // finished = true (driver must transition to next step)
            }
        }
    }
}
