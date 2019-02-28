#include "Autonomous/Steps/RotateIntake.h"
#include "Robot.h"

RotateIntake::RotateIntake(IntakeRotate::IntakePosition _target, bool waitForPosition, double timeout)
  : targetPosition(_target), waitForPosition(waitForPosition), timeout(timeout) {}

bool RotateIntake::Run(std::shared_ptr<World> world) {
    if (startTime < 0) {
        startTime = frc::Timer::GetFPGATimestamp();
        Robot::intakeRotate->SetIntakePosition(targetPosition);
    }
    const double elapsed = frc::Timer::GetFPGATimestamp() - startTime;

    // FIXME: Add get position to intake 
    if (waitForPosition && (elapsed < timeout)) {
        return false;
    } else {
        return true;
    }
}