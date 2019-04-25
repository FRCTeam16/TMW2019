/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Lift/ThirdStep.h"
#include <iostream>
#include "Robot.h"
#include "Subsystems/JackScrews.h"


void Thirdstep::Execute() {
    auto jackScrewControls = Robot::jackScrews->GetJackScrewControls();
    auto wheels = Robot::driveBase->GetWheels();

    if (IsFirstRun()) {
        std::cout << "Lift Third Step\n";
        Robot::jackScrews->ShiftFront(JackScrews::ShiftMode::kDrive);
        wheels.RL->SetDriveSoftMinMaxOutput(-1.0, 0.0);
        wheels.RR->SetDriveSoftMinMaxOutput(-1.0, 0.0);

        Robot::jackScrews->ConfigureControlled(
            JackScrews::LiftMode::kBack,
            JackScrews::Direction::kUp,
            JackScrewControl::EndStateAction::kSwitchToAmpDetect);

        wheels.FL->UseOpenLoopDrive(0.0);
        wheels.FR->UseOpenLoopDrive(0.0);
        wheels.FL->SetDriveBrakeMode();
        wheels.FR->SetDriveBrakeMode();
        jackScrewControls->FL->SetControlSpeed(0.0);
        jackScrewControls->FR->SetControlSpeed(0.0);
        startTime = frc::Timer::GetFPGATimestamp();
    } else {
        if (!liftFinished) {
            bool leftFinished = jackScrewControls->RL->IsFinished();
            bool rightFinished = jackScrewControls->RR->IsFinished();
            bool timedOut = (frc::Timer::GetFPGATimestamp() - startTime) >= 5.0;
            liftFinished = (leftFinished || rightFinished) || timedOut;

            // Turn off outputs
            if (liftFinished) {
                std::cout << "Setting rear wheels to hold open and finish\n";
                jackScrewControls->RL->HoldOpenAndFinish();
                jackScrewControls->RR->HoldOpenAndFinish();
            }
        } else {
                Robot::driveBase->SetTargetAngle(-180.0);

                const double autoDriveSpeed = BSPrefs::GetInstance()->GetDouble("Lift.step2.drivespeed.y", 0.3);;
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


