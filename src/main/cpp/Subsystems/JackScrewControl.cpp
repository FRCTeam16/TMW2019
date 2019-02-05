#include "Subsystems/JackScrewControl.h"

void JackScrewControl::Init(int targetDistance_, int controlTimeStart_) {
    targetDistance = targetDistance_;
    controlTimeStart = controlTimeStart_;
    startPosition = currentPosition = lastPosition = wheel->GetDriveEncoderPosition();
    currentState == JackScrewState::kOpenLoop;
}

void JackScrewControl::Run() {
    if (JackScrewState::kSwerve == currentState) {
        return; // not controllable by us
    }
    const double now = frc::Timer::GetFPGATimestamp();
    const double speed = RampUtil::RampUp(controlSpeed, (now - controlTimeStart), jackScrewRampTime, 0.0);

    currentPosition = wheel->GetDriveEncoderPosition();
    lastChange = abs(currentPosition - lastPosition);
    accumulatedPosition += lastChange;
    int remaining = abs(targetDistance - accumulatedPosition);      // TODO: Sign problem here

    if (remaining < rotationCloseLoopThreshold) {
        currentState = JackScrewState::kClosedLoop;
    }

    switch (currentState) {
        case JackScrewState::kOpenLoop:
            wheel->UseOpenLoopDrive(speed);
            break;
        case JackScrewState::kClosedLoop:
            std::cout << " *** CLOSED LOOP ***\n";
            wheel->UseClosedLoopDrive(GetTargetDistance()); 
            break;
        default:
            std::cout << "!!! Warning : JackScrew in swerve state but run() reaching control !!!\n";
    }
    lastPosition = currentPosition;
}

void JackScrewControl::Hold() {
    currentState = JackScrewState::kClosedLoop;
}