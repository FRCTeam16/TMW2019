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
    auto wheels = Robot::driveBase->GetWheels();

    if (IsFirstRun()) {
        std::cout << "Lift Third Step\n";
        Robot::jackScrews->ConfigureControlled(
            JackScrews::LiftMode::kBack,
            JackScrews::Direction::kUp,
            JackScrewControl::EndStateAction::kSwitchToAmpDetect);


            wheels.FL->SetDriveBrakeMode();
            wheels.FR->SetDriveBrakeMode();
            jackScrewControls->FL->SetControlSpeed(0.0);
            jackScrewControls->FR->SetControlSpeed(0.0);

            jackScrewControls->FL->Run();
            jackScrewControls->FR->Run();
    } else {
        if (!liftFinished) {
            bool leftFinished = jackScrewControls->RL->IsFinished();
            bool rightFinished = jackScrewControls->RR->IsFinished();
            liftFinished = leftFinished && rightFinished;
            
            jackScrewControls->FL->Run();
            jackScrewControls->FR->Run();
        } else {
            if (!shiftedToSwerve) {
                // Safety set output
                jackScrewControls->FL->SetControlSpeed(0.0);
                jackScrewControls->FR->SetControlSpeed(0.0);
                jackScrewControls->RL->SetControlSpeed(0.0);
                jackScrewControls->RR->SetControlSpeed(0.0);
                Robot::jackScrews->ShiftAll(JackScrews::ShiftMode::kDrive);

                // TODO: Put drive wheels in brake mode
                
                wheels.FL->SetDriveBrakeMode();
                wheels.FR->SetDriveBrakeMode();
                wheels.RL->SetDriveBrakeMode();
                wheels.RR->SetDriveBrakeMode();
                shiftedToSwerve = true;
                Robot::driveBase->SetTargetAngle(-180.0);

                jackScrewControls->FL->Run();
                jackScrewControls->FR->Run();
            } else {
                wheels.FL->SetDriveCoastMode();
                wheels.FR->SetDriveCoastMode();
                wheels.RL->SetDriveCoastMode();
                wheels.RR->SetDriveCoastMode();
                Robot::driveBase->SetTargetAngle(-180.0);
                double leftInput = Robot::oi->GetJoystickX();
                double rightInput = Robot::oi->GetJoystickY();
                std::cout << "Left: " << leftInput << " | Right: " << rightInput << "\n";
 
                if (Robot::oi->DL9->Pressed()) {
                    const double crabSpeed = PrefUtil::getSet("Lift.step3.drivespeed.y", -0.2);
                    Robot::driveBase->Crab(
                        Robot::driveBase->GetCrabTwistOutput(),
                        -crabSpeed, 0, false);
                } else {
                    double twist = Robot::oi->GetJoystickTwist(0.10);
                    Robot::driveBase->Crab(twist, -rightInput, leftInput, true);
                }
            }  
        }
    }
}


