/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Lift/Thirdstep.h"
#include <iostream>
#include"Robot.h"
#include "Subsystems/JackScrews.h"


void Thirdstep::Execute() {
    auto jackScrewControls = Robot::jackScrews->GetJackScrewControls();

    if (IsFirstRun()) {
        std::cout << "Lift Third Step\n";
        Robot::jackScrews->ConfigureControlled(
            JackScrews::LiftMode::kBack,
            JackScrews::Direction::kUp,
            JackScrewControl::EndStateAction::kSwitchToAmpDetect);
    } else {
        if (!liftFinished) {
            bool leftFinished = jackScrewControls->RL->IsFinished();
            bool rightFinished = jackScrewControls->RR->IsFinished();
            liftFinished = leftFinished && rightFinished;
        } else {
            // Safety set output
            jackScrewControls->FL->SetControlSpeed(0.0);
            jackScrewControls->FR->SetControlSpeed(0.0);
            jackScrewControls->RL->SetControlSpeed(0.0);
            jackScrewControls->RR->SetControlSpeed(0.0);

            Robot::jackScrews->ShiftAll(JackScrews::ShiftMode::kDrive);
            
            // TODO: allow exit to robot mode finished = true;
        }
    }
}


