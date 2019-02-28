/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Autonomous/Steps/DoIntakeAction.h"
#include "Robot.h"

DoIntakeAction::DoIntakeAction(Action action, double timeToRun) 
  : action(action), timeToRun(timeToRun) {}

bool DoIntakeAction::Run(std::shared_ptr<World> world) {
    if (startTime < 0) {
        startTime = frc::Timer::GetFPGATimestamp();
        if (Action::kEjectHatch == action) {
            Robot::intake->EjectHatch();
        } else if (Action::kIntakeHatch == action) {
            Robot::intake->IntakeHatch();
        }
    }
    const double elapsed = frc::Timer::GetFPGATimestamp() - startTime;
    return elapsed > timeToRun;
}
