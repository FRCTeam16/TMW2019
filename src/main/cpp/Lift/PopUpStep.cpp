#include "Lift/PopUpStep.h"
#include "Robot.h"


PopUpStep::PopUpStep() = default;

void PopUpStep::Execute() {
    auto jackScrewControls = Robot::jackScrews->GetJackScrewControls();
    const double elapsed = frc::Timer::GetFPGATimestamp() - startTime;

    if (IsFirstRun()) {
        std::cout << "PopUpStep Shifting\n";
        auto wheels = Robot::driveBase->GetWheels();
        wheels.FL->SetDriveSoftMinMaxOutput(0.0, 1.0);
        wheels.FR->SetDriveSoftMinMaxOutput(0.0, 1.0);
        wheels.RL->SetDriveSoftMinMaxOutput(0.0, 1.0);
        wheels.RR->SetDriveSoftMinMaxOutput(0.0, 1.0);

        Robot::crawler->SetCrawlEnabled(false);
        Robot::jackScrews->ShiftAll(JackScrews::ShiftMode::kJackscrews);
        Robot::jackScrews->ConfigureOpenLoop(0.0);
        Robot::jackScrews->SetDoPopClimb();
    } else {
        const double now = frc::Timer::GetFPGATimestamp();
        const double delta = (now - startTime);
        const double shiftDelay = 0.250;  // delay for dog

        if (delta > shiftDelay) {
            std::cout << "PopUpStep: Running Controlled\n";
            if (firstAfterShifting) {
                std::cout << "PopUpStep: first after shifting\n";

                firstAfterShifting = false;
                Robot::jackScrews->ConfigureControlled(
                    JackScrews::LiftMode::kAll,
                    JackScrews::Direction::kDown,
                    JackScrewControl::EndStateAction::kSwitchToControl);
            }
        } else {
            std::cout << "now: " << now << " | startTime: " << startTime << " | delta: " << delta << "\n";
            jackScrewControls->FL->SetControlSpeed(0.0);
            jackScrewControls->FR->SetControlSpeed(0.0);
            jackScrewControls->RL->SetControlSpeed(0.0);
            jackScrewControls->RR->SetControlSpeed(0.0);
        }

    }

}
