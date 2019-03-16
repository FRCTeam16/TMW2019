/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Lift/FirstStep.h"
#include "Robot.h"
#include "Subsystems/JackScrews.h"
#include "Subsystems/Drive/TMW2019SwerveWheel.h"

FirstStep::FirstStep() = default;

void FirstStep::Execute() {
    auto jackScrewControls = Robot::jackScrews->GetJackScrewControls();
    const double elapsed = frc::Timer::GetFPGATimestamp() - startTime;

    if (IsFirstRun()) {
        std::cout << "JackScrew First Step Shifting\n";
        auto wheels = Robot::driveBase->GetWheels();
        wheels.FL->SetDriveSoftMinMaxOutput(0.0, 1.0);
        wheels.FR->SetDriveSoftMinMaxOutput(0.0, 1.0);
        wheels.RL->SetDriveSoftMinMaxOutput(0.0, 1.0);
        wheels.RR->SetDriveSoftMinMaxOutput(0.0, 1.0);

        Robot::crawler->SetCrawlEnabled(false);
        Robot::jackScrews->ShiftAll(JackScrews::ShiftMode::kJackscrews);
        Robot::jackScrews->ConfigureOpenLoop(0.0);
    } else {
        const double now = frc::Timer::GetFPGATimestamp();
        const double delta = (now - startTime);
        const double shiftDelay = 0.250;                  // delay for dog

        if (delta > shiftDelay) {
            std::cout << "JackScrew First Step Running Controlled\n";
            if (firstAfterShifting) {
                firstAfterShifting = false;
                Robot::jackScrews->ConfigureControlled(JackScrews::LiftMode::kAll, JackScrews::Direction::kDown, JackScrewControl::EndStateAction::kSwitchToControl, true);    //
                std::cout << "Configured wheels for control\n";
            }
        } else {
            std::cout << "now: " << now << " | startTime: " << startTime << " | delta: " << delta << "\n";
            jackScrewControls->FL->SetControlSpeed(0.0);
            jackScrewControls->FR->SetControlSpeed(0.0);
            jackScrewControls->RL->SetControlSpeed(0.0);
            jackScrewControls->RR->SetControlSpeed(0.0);
        }
    }

    finished =  (jackScrewControls->FL->GetCurrentState() == JackScrewControl::JackScrewState::kClosedLoop) &&
                (jackScrewControls->FR->GetCurrentState() == JackScrewControl::JackScrewState::kClosedLoop) &&
                (jackScrewControls->RL->GetCurrentState() == JackScrewControl::JackScrewState::kClosedLoop) &&
                (jackScrewControls->RR->GetCurrentState() == JackScrewControl::JackScrewState::kClosedLoop);
    if (finished) {
        std::cout << "First Step detected step finished\n";
    }
    /// Re-enable crawler on finish or timeout
    if (finished || (elapsed > 1.0) ) {
        Robot::crawler->SetCrawlEnabled(true);
    }
}

