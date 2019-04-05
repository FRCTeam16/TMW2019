/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Lift/LiftController.h"
#include "Lift/FirstStep.h"
#include "Lift/SecondStep.h"
#include "Lift/ThirdStep.h"
#include "Robot.h"

LiftController::LiftController() = default;

void LiftController::Next() {
    // TODO: think about protected code blocks and an "allowed to transition" state for guards
    
    std::cout << "LiftController::Run() transitioning to next state\n";
    currentState = static_cast<LiftState>(static_cast<int>(currentState) + 1);

    switch (currentState) {
        case LiftState::kNone:
            break;
        case LiftState::kFirst:
            std::cout << "Setting First Step\n";
            currentAction.reset(new FirstStep());
            break;
        case LiftState::kSecond:
            std::cout << "Setting Second Step\n";
            currentAction.reset(new SecondStep());
            break;
        case LiftState::kThird:
            std::cout << "Setting Third Step\n";
            currentAction.reset(new Thirdstep());
            break;
        case LiftState::kFinished:
            std::cout << "*** Resetting current action ***\n";
            currentAction.reset();
    }
}


void LiftController::Run() {
    if (currentAction.get() == nullptr) {
        // std::cout << "!!! No current action in Lift Controller\n";
        return;
    }

    // TODO: Maybe always run
    if (!currentAction->IsFinished()) {
        currentAction->Run();
    }
}

bool LiftController::IsRunning() {
    return currentAction.get() != nullptr;
}
