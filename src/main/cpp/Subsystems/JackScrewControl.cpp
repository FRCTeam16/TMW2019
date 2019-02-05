#include "Subsystems/JackScrewControl.h"

void JackScrewControl::Init(int targetDistance_, int controlTimeStart_) {
    std::cout << "JackScrewControl::Init()\n";
    targetDistance = targetDistance_;
    controlTimeStart = controlTimeStart_;
    startPosition = currentPosition = lastPosition = wheel->GetDriveEncoderPosition();
    accumulatedPosition = 0.0;
    SetCurrentState(JackScrewState::kOpenLoop);
    controlSpeed = 0.0;
    autoSwitchToControl = true;
    std::cout << "jackScrews->FL: " << this << " : " << static_cast<int>(GetCurrentState()) << "\n";
}

void JackScrewControl::InitOpenLoop(double speed) {
    SetCurrentState(JackScrewState::kOpenLoop);
    controlSpeed = speed;
    autoSwitchToControl = false;
}

void JackScrewControl::Run() {
    if (JackScrewState::kSwerve == GetCurrentState()) {
        return; // not controllable by us
    }

    const double now = frc::Timer::GetFPGATimestamp();
    const double speed = RampUtil::RampUp(controlSpeed, (now - controlTimeStart), jackScrewRampTime, 0.0);

    currentPosition = wheel->GetDriveEncoderPosition();
    lastChange = abs(currentPosition - lastPosition);
    accumulatedPosition += lastChange;
    int remaining = abs(targetDistance - accumulatedPosition);      // TODO: Sign problem here

    if (autoSwitchToControl && (currentState != JackScrewState::kClosedLoop) && (remaining < rotationCloseLoopThreshold)) {
        std::cout << " JackScrewControl::Run flipping to closed loop\n";
        SetCurrentState(JackScrewState::kClosedLoop);
    }

    switch (GetCurrentState()) {
        case JackScrewState::kOpenLoop:
            wheel->UseOpenLoopDrive(speed);
            break;
        case JackScrewState::kClosedLoop:
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