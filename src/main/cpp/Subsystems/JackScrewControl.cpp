#include <Robot.h>
#include "Subsystems/JackScrewControl.h"


void JackScrewControl::ConfigureControlled(double targetDistance_, double controlTimeStart_, EndStateAction action) {
    std::cout << "JackScrewControl::Init()\n";
    targetDistance = targetDistance_;
    controlTimeStart = controlTimeStart_;
    startPosition = currentPosition = lastPosition = wheel->GetDriveEncoderPosition();
    accumulatedPosition = 0.0;
    SetCurrentState(JackScrewState::kOpenLoop);
    controlSpeed = 0.0;
    endStateAction = action;
    firstThresholdRun = true;
    finished = false;
    ampDetector.Reset();
}

void JackScrewControl::InitOpenLoop(double speed, EndStateAction action) {
    SetCurrentState(JackScrewState::kOpenLoop);
    controlSpeed = speed;
    endStateAction = action;
}

void JackScrewControl::Run() {
    if (JackScrewState::kSwerve == GetCurrentState()) {
        return; // not controllable by us
    }

    const double now = frc::Timer::GetFPGATimestamp();
    double speed = RampUtil::RampUp(controlSpeed, (now - controlTimeStart), jackScrewRampTime, 0.0);

    currentPosition = wheel->GetDriveEncoderPosition();
    lastChange = abs(currentPosition - lastPosition);
    accumulatedPosition += lastChange;
    const double remaining = fabs(targetDistance - accumulatedPosition);
    const bool inThreshold = remaining < rotationCloseLoopThreshold;

    switch (endStateAction) {
        case EndStateAction::kNone:
            break;
        case EndStateAction::kSwitchToAmpDetect:
            if (inThreshold) {
                if (firstThresholdRun) {
                    Robot::jackScrews->ConfigureOpenLoop(kJackScrewApproachSpeed, EndStateAction::kSwitchToAmpDetect); // calls back and modifies our state
                    firstThresholdRun = false;
                }
                if (!ampDetector.Check()) {
                    ampDetector.AddValue(wheel->GetDriveOutputCurrent());
                } else {
                    std::cout << "JackScrewControl::Run detected amp spike\n";
                    SetControlSpeed(0.0);
                    speed = 0.0;
                    finished = true;
                }
            }
            break;

        case EndStateAction::kSwitchToControl:
            if (!IsClosedLoop() && inThreshold) {
                std::cout << " JackScrewControl::Run flipping to closed loop\n";
                SetCurrentState(JackScrewState::kClosedLoop);
                finished = true;
            }
            break;
    }

    switch (GetCurrentState()) {
        case JackScrewState::kOpenLoop:
            wheel->UseOpenLoopDrive(speed);
            break;
        case JackScrewState::kClosedLoop:
            wheel->UseClosedLoopDrive(GetTargetDistance());     // FIXME? only works positive direction
            break;
        default:
            std::cout << "!!! Warning : JackScrew in swerve state but run() reaching control !!!\n";
    }
    lastPosition = currentPosition;
}

void JackScrewControl::Hold() {
    currentState = JackScrewState::kClosedLoop;
}