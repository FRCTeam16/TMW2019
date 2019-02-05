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


SecondStep::SecondStep() {}

void SecondStep::Execute() {
    const double kThreshold = 0.1;
    auto jackScrewControls = Robot::jackScrews->GetJackScrewControls();


    if (IsFirstRun()) {
        std::cout << "Lift Second Step\n";
        Robot::jackScrews->ConfigureControlled(JackScrews::LiftMode::kFront, JackScrews::Position::kUp);
    } else {
        finished =  (jackScrewControls->FL->GetCurrentState() == JackScrewControl::JackScrewState::kClosedLoop) &&
                    (jackScrewControls->FR->GetCurrentState() == JackScrewControl::JackScrewState::kClosedLoop);
    }
}
