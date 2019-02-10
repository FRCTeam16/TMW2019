/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Lift/LiftController.h"
#include "Lift/FirstStep.h"
#include "Lift/SecondStep.h"
#include "Lift/Thirdstep.h"
#include "Robot.h"

LiftController::LiftController() = default;

void LiftController::Next() {
    // TODO: think about protected code blocks and an "allowed to transition" state for guards
    if (currentAction.get() == nullptr) {
        stateTransitioned = false;
        switch (currentState) {
            case LiftState::kNone:
                currentAction.reset(new FirstStep());
                break;
            case LiftState::kFirstStep:
                currentAction.reset(new SecondStep());
                break;
            case LiftState::kSecondStep:
                currentAction.reset(new Thirdstep());
                break;
            case LiftState::kFinished:
                std::cout << "*** Resetting current action ***\n";
                currentAction.reset();
        }
    }
}


void LiftController::Run() {
    if (currentAction.get() == nullptr) {
        // std::cout << "!!! No current action in Lift Controller\n";
        return;
    }

    if (!currentAction->IsFinished()) {
        currentAction->Run();
    } else if (!stateTransitioned) {
        std::cout << "LiftController::Run() transitioning to next state\n";
        currentState = static_cast<LiftState>(static_cast<int>(currentState) + 1);
        stateTransitioned = true;
    }
}

bool LiftController::IsRunning() {
    return currentAction.get() == nullptr;
}
