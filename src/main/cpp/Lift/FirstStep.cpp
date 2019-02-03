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
        std::cout << "JackScrew First Step Shifting\n";
        Robot::jackScrews->ShiftAll(JackScrews::ShiftMode::kJackscrews);
        Robot::jackScrews->ConfigureControlled(JackScrews::LiftMode::kAll, JackScrews::Position::kDown);
    } else {
        const double now = frc::Timer::GetFPGATimestamp();
        const double delta = (now - startTime);
        const double shiftDelay = 0.250;                  // delay for dog

        if (delta > shiftDelay) {
            std::cout << "JackScrew First Step Running Controlled\n";
            Robot::jackScrews->Run();
        } else {
            std::cout << "now: " << now << " | startTime: " << startTime << " | delta: " << delta << "\n";
        }
    }
}

