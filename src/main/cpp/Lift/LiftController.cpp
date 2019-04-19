#include "Lift/LiftController.h"
#include "Lift/FirstStep.h"
#include "Lift/SecondStep.h"
#include "Lift/ThirdStep.h"
#include "Lift/PopUpStep.h"
#include "Lift/PopDownStep.h"
#include "Robot.h"

LiftController::LiftController() = default;

void LiftController::Next() {
    // TODO: think about protected code blocks and an "allowed to transition" state for guards
    std::cout << "LiftController::Run() transitioning to next state\n";

    if (normalClimb) {
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
    } else {
        // Performing eight inch climb sequence
        popCurrentState = static_cast<PopLiftState>(static_cast<int>(popCurrentState) + 1);

        switch (popCurrentState) {
            case PopLiftState::kNone:
                break;
            case PopLiftState::kFirst:
                std::cout << "Setting First Step (Pop)\n";
                currentAction.reset(new PopUpStep());
                break;
            case PopLiftState::kSecond:
                std::cout << "Setting First Step (Pop)\n";
                currentAction.reset(new PopDownStep());
                break;
            case PopLiftState::kFinished:
                std::cout << "*** Resetting current action (Hop) ***\n";
                currentAction.reset();
        }
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
