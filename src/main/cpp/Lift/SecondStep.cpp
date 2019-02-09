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
                // set direction
                shiftedFrontToSwerve = true;
                std::cout << "TODO: Start Tanking\n";
            } else {
                double leftInput = Robot::oi->getDriverLeft()->GetY();
                double rightInput = Robot::oi->getDriverRight()->GetY();
                std::cout << "Left: " << leftInput << " | Right: " << rightInput << "\n";
            }
        }
    }
}
