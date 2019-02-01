/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Lift/LiftController.h"
#include "Lift/FirstStep.h"

LiftController::LiftController() {}

void LiftController::Next() {
    if (currentAction.get() == nullptr || currentAction->IsFinished()) {
        switch (currentState) {
            case LiftState::kNone:
                currentAction.reset(new FirstStep());
                break;
            case LiftState::kLiftUp:
                break;
            case LiftState::kFirstDrive:
                break;
            case LiftState::kLastDrive:
                break;
            case LiftState::kFinished:
                break;
        }
    }
}

void LiftController::Run() {
    if (!currentAction->IsFinished()) {
        currentAction->Run();
    }
}
