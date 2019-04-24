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

    // auto wheels = Robot::driveBase->GetWheels();
    // DriveInfo<rev::CANSparkMax*> sparks;
    // sparks.FL = static_cast<TMW2019SwerveWheel*>(wheels.FL.get())->GetDriveMotor().get();
    // sparks.FR = static_cast<TMW2019SwerveWheel*>(wheels.FR.get())->GetDriveMotor().get();
    // sparks.RL = static_cast<TMW2019SwerveWheel*>(wheels.RL.get())->GetDriveMotor().get();
    // sparks.RR = static_cast<TMW2019SwerveWheel*>(wheels.RR.get())->GetDriveMotor().get();

    DriveInfo<bool> jscDone {false};
    jscDone.FL = (jackScrewControls->FL->GetCurrentState() == JackScrewControl::JackScrewState::kClosedLoop); // || sparks.FL->IsFollower();
    jscDone.FR = (jackScrewControls->FR->GetCurrentState() == JackScrewControl::JackScrewState::kClosedLoop); // || sparks.FR->IsFollower();
    jscDone.RL = (jackScrewControls->RL->GetCurrentState() == JackScrewControl::JackScrewState::kClosedLoop); // || sparks.RL->IsFollower();;
    jscDone.RR = (jackScrewControls->RR->GetCurrentState() == JackScrewControl::JackScrewState::kClosedLoop); // || sparks.RR->IsFollower();;

    finished =  jscDone.FL && jscDone.FR && jscDone.RL && jscDone.RR;

    if (finished) {
        std::cout << "First Step detected step finished\n";
    } else {
        //
        // Check axes to make sure we haven't experienced slippage
        //  
        
        // // const bool frontFollowing = (sparks.FL->IsFollower() || sparks.FR->IsFollower());
        // // if (frontFollowing) {
        // //     std::cout << "~~~ Front already running a follower\n";
        // // }
        // if ((jscDone.FL ^ jscDone.FR) && !frontFollowing) {
        //     std::cout << "~~~ Detected xor front axis climb state - probable slippage\n";
            
        //     if (jackScrewControls->FL->GetAccumulatedPosition() < jackScrewControls->FR->GetAccumulatedPosition()) {
        //         // Assume FR has slipped and is reporting incorrect values
        //         // make it open loop and a follower
        //         if (!sparks.FR->IsFollower()) {
        //             std::cout << "...Setting FR to follow FL\n";
        //             jackScrewControls->FR->SetCurrentState(JackScrewControl::JackScrewState::kSwerve);  // disables our controlled input
        //             // sparks.FR->Follow(*sparks.FL, false);
        //             sparks.FR->Follow(rev::CANSparkMax::kFollowerSparkMax, sparks.FL->GetDeviceId(), false );
        //         } else {
        //             std::cout << "...FR already a follower\n";
        //         }
        //     } else {
        //         // Assume FL has slipped and is reporting incorrect values
        //         if (!sparks.FL->IsFollower()) {
        //             std::cout << "...Setting FL to follow FR\n";
        //             jackScrewControls->FL->SetCurrentState(JackScrewControl::JackScrewState::kSwerve);  // disables our controlled input
        //             // sparks.FL->Follow(*sparks.FR, false);
        //             sparks.FL->Follow(rev::CANSparkMax::kFollowerSparkMax, sparks.FR->GetDeviceId(), false );
        //         } else {
        //             std::cout << "...FL already a follower\n";
        //         }
        //     }
        // }

        // const bool backFollowing = (sparks.RL->IsFollower() || sparks.RR->IsFollower());
        // if (backFollowing) {
        //     std::cout << "~~~ Back already running a follower\n";
        // }
        // if ((jscDone.RL ^ jscDone.RR) && !backFollowing) {
        //     std::cout << "~~~ Detected xor rear axis climb state - probable slippage\n";
        //     if (jackScrewControls->RL->GetAccumulatedPosition() < jackScrewControls->RR->GetAccumulatedPosition()) {
        //         // Assume RR has slipped and is reporting incorrect values
        //         // make it open loop and a follower
        //         if (!sparks.RR->IsFollower()) {
        //             std::cout << "...Setting RR to follow RL\n";
        //             jackScrewControls->RR->SetCurrentState(JackScrewControl::JackScrewState::kSwerve);  // disables our controlled input
        //             // sparks.RR->Follow(*sparks.RL);
        //             sparks.RR->Follow(rev::CANSparkMax::kFollowerSparkMax, sparks.RL->GetDeviceId(), false );
        //         } else {
        //             std::cout << "...RR already a follower\n";
        //         }
        //     } else {
        //         // Assume RL has slipped and is reporting incorrect values
        //         if (!sparks.RL->IsFollower()) {
        //             std::cout << "...Setting RL to follow RR\n";
        //             jackScrewControls->RL->SetCurrentState(JackScrewControl::JackScrewState::kSwerve);  // disables our controlled input
        //             // sparks.RL->Follow(*sparks.RR);
        //             sparks.RL->Follow(rev::CANSparkMax::kFollowerSparkMax, sparks.RR->GetDeviceId(), false );
        //         } else {
        //             std::cout << "...RL already a follower\n";
        //         }
                
        //     }
        // }

    }
    /// Re-enable crawler on finish or timeout
    if (finished || (elapsed > 4.0) ) {
        if (!crawlerStarted) {
            
        }
        Robot::crawler->SetCrawlEnabled(true);
        Robot::crawler->Back();

    }
}

