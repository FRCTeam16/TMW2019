#include "Subsystems/JackScrewCalculator.h"

void JackScrewCalculator::Run() {
    const double now = frc::Timer::GetFPGATimestamp();
    const double speed = RampUtil::RampUp(controlSpeed, (now - controlTimeStart), jackScrewRampTime, 0.0);
    // const double speed = controlSpeed;
    // std::cout << "JSCalc-OpenLoop Control: " << controlSpeed << " | Ramp: " << speed << "\n";

    currentPosition = wheel->GetDriveEncoderPosition();
    lastChange = abs(currentPosition - lastPosition);
    accumulatedPosition += lastChange;
    int remaining = abs(targetDistance - accumulatedPosition);      // TODO: Sign problem here

    if (remaining < rotationCloseLoopThreshold) {
        std::cout << " *** CLOSED LOOP ***\n";
        wheel->UseClosedLoopDrive(GetTargetDistance()); 
        closedLoop = true;
    } else {
        wheel->UseOpenLoopDrive(speed);
    }
    lastPosition = currentPosition;
}

void JackScrewCalculator::Hold() {
    wheel->UseClosedLoopDrive(GetTargetDistance()); 
    closedLoop = true;
}