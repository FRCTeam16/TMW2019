#include "Lift/PopDownStep.h"
#include "Robot.h"

void PopDownStep::Execute() {
    auto jackScrewControls = Robot::jackScrews->GetJackScrewControls();
    auto wheels = Robot::driveBase->GetWheels();

    if (IsFirstRun()) {
        std::cout << "PopDownStep\n";
        wheels.FL->SetDriveSoftMinMaxOutput(-1.0, 0.0);
        wheels.FR->SetDriveSoftMinMaxOutput(-1.0, 0.0);
        wheels.RL->SetDriveSoftMinMaxOutput(-1.0, 0.0);
        wheels.RR->SetDriveSoftMinMaxOutput(-1.0, 0.0);

        Robot::jackScrews->ConfigureControlled(
            JackScrews::LiftMode::kAll,
            JackScrews::Direction::kUp,
            JackScrewControl::EndStateAction::kSwitchToAmpDetect);
    } else {
        if (!liftFinished) {
            bool timedOut = (frc::Timer::GetFPGATimestamp() - startTime) >= 5.0;

            bool flFinished = jackScrewControls->FL->IsFinished();
            bool frFinished = jackScrewControls->FR->IsFinished();
            bool rlFinished = jackScrewControls->RL->IsFinished();
            bool rrFinished = jackScrewControls->RR->IsFinished();

            bool frontFinished = false;
            bool backFinished = false;

            if (flFinished || frFinished) {
                jackScrewControls->FL->HoldOpenAndFinish();
                jackScrewControls->FR->HoldOpenAndFinish();
                frontFinished = true;
            }
            if (rlFinished || rrFinished) {
                jackScrewControls->RL->HoldOpenAndFinish();
                jackScrewControls->RR->HoldOpenAndFinish();
                backFinished = true;
            }

            liftFinished = (frontFinished && backFinished) || timedOut;
        } else {
            // Finished climbing
            std::cout << "PopDownStep Finished\n";
            wheels.FL->SetDriveSoftMinMaxOutput(-1.0, 1.0);
            wheels.FR->SetDriveSoftMinMaxOutput(-1.0, 1.0);
            wheels.RL->SetDriveSoftMinMaxOutput(-1.0, 1.0);
            wheels.RR->SetDriveSoftMinMaxOutput(-1.0, 1.0);
            finished = true;
        }
    }
}
