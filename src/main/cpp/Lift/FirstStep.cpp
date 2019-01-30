/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Lift/FirstStep.h"
#include "Robot.h"
#include "Subsystems/JackScrews.h"

FirstStep::FirstStep() {
}

void FirstStep::Execute() {
    if (IsFirstRun()) {
        // Robot::jackScrews->ShiftAll(JackScrews::ShiftMode::kJackscrews);
    } else {
        const double now = frc::Timer::GetFPGATimestamp();
        const double delta = (now - startTime);
        const double shiftDelay = 250;                  // delay for dog

        if (delta > shiftDelay) {
            // Robot::jackScrews->RunControlled(JackScrews::LiftMode::kAll, JackScrews::Position::kDown);
        }
    }
}
