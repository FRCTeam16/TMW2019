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
#include "Subsystems/Drive/TMW2019SwerveWheel.h"


void SecondStep::Execute() {
    const double kThreshold = 0.1;
    auto jackScrewControls = Robot::jackScrews->GetJackScrewControls();
    auto wheels = Robot::driveBase->GetWheels();

    if (IsFirstRun()) {
        Robot::crawler->Stop();
        wheels.FL->SetDriveSoftMinMaxOutput(-1.0, 0.0);
        wheels.FR->SetDriveSoftMinMaxOutput(-1.0, 0.0);
        // rl & rr in control mode

        // // Disable follower mode
        // DriveInfo<rev::CANSparkMax*> sparks;
        // sparks.FL = static_cast<TMW2019SwerveWheel*>(wheels.FL.get())->GetDriveMotor().get();
        // sparks.FR = static_cast<TMW2019SwerveWheel*>(wheels.FR.get())->GetDriveMotor().get();
        // sparks.RL = static_cast<TMW2019SwerveWheel*>(wheels.RL.get())->GetDriveMotor().get();
        // sparks.RR = static_cast<TMW2019SwerveWheel*>(wheels.RR.get())->GetDriveMotor().get();

        // sparks.FL->Follow(rev::CANSparkMax::kFollowerDisabled, 0);
        // sparks.FR->Follow(rev::CANSparkMax::kFollowerDisabled, 0);
        // sparks.RL->Follow(rev::CANSparkMax::kFollowerDisabled, 0);
        // sparks.RR->Follow(rev::CANSparkMax::kFollowerDisabled, 0);


        std::cout << "Lift Second Step\n";
        Robot::jackScrews->ConfigureControlled(
                JackScrews::LiftMode::kFront,
                JackScrews::Direction::kUp,
                JackScrewControl::EndStateAction::kSwitchToAmpDetect);
        startTime = frc::Timer::GetFPGATimestamp();
    } else {
        if (!liftFinished) {
            // Waiting for front jackscrew amp detection to kickout
            bool leftFinished = jackScrewControls->FL->IsFinished();
            bool rightFinished = jackScrewControls->FR->IsFinished();
            bool timedOut = (frc::Timer::GetFPGATimestamp() - startTime) >= 5.0;
            std::cout << "LiftFinished Check: left? " << leftFinished
                                        << " | right? " << rightFinished
                                        << " | timedOut? " << timedOut << "\n";
            liftFinished = (leftFinished || rightFinished) || timedOut;
        } else {
            if (!shiftedFrontToSwerve) {
                Robot::jackScrews->ShiftFront(JackScrews::ShiftMode::kDrive);
                Robot::jackScrews->SetLiftMode(JackScrews::LiftMode::kNone);

                wheels.FL->SetDriveSoftMinMaxOutput(-1.0, 1.0);
                wheels.FR->SetDriveSoftMinMaxOutput(-1.0, 1.0);

                Robot::driveBase->SetTargetAngle(-180.0);
                shiftStartTime = frc::Timer::GetFPGATimestamp();
                shiftedFrontToSwerve = true;
            } else {

                // Have some time to make sure we are shifted
                if ((frc::Timer::GetFPGATimestamp() - shiftStartTime) < 0.25) {
                    std::cout << "SecondStep - waiting for shift before driving\n";
                    liftDrive.DriveTank(0.0, 0.0);
                    return;
                }

                Robot::driveBase->SetTargetAngle(-180.0);

                // As if joystick being pulled toward user
                const double autoDriveSpeed = BSPrefs::GetInstance()->GetDouble("Lift.step2.drivespeed.y", 0.3);
                double leftInput = autoDriveSpeed;
                double rightInput = autoDriveSpeed;

                bool useHumanInput = false;
                if (useHumanInput) {
                    const double kLowJoyThreshold = 0.15;
                    const double kHighJoyThreshold = 0.30;
                    leftInput = Robot::oi->getDriverLeft()->GetY();
                    rightInput = Robot::oi->getDriverRight()->GetY();

                    int dir = leftInput < 0 ? -1 : 1;
                    if (fabs(leftInput) < kLowJoyThreshold) { leftInput = 0.0; }
                    if (fabs(leftInput) > kHighJoyThreshold) { leftInput = kHighJoyThreshold * dir; }

                    dir = rightInput < 0 ? -1 : 1;
                    if (fabs(rightInput) < kLowJoyThreshold) { rightInput = 0.0; }
                    if (fabs(rightInput) > kHighJoyThreshold) { rightInput = kHighJoyThreshold * dir; }
                }
                
                std::cout << "Left: " << leftInput << " | Right: " << rightInput << "\n";
 
                if (Robot::oi->DL9->Pressed()) {
                    // const double crabSpeed = BSPrefs::GetInstance()->GetDouble("Lift.step2.drivespeed.y", -0.2);
                    // liftDrive.DriveFront(Robot::driveBase->GetCrabTwistOutput(), crabSpeed, 0, true);
                } else {
                    // swap inputs
                    liftDrive.DriveTank(-rightInput, -leftInput);   // invert direction
                }
            }
        }
    }
}
