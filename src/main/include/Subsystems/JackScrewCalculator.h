#pragma once

#include "frc/Preferences.h"
#include "frc/Timer.h"
#include "Subsystems/Drive/SwerveWheel.h"
#include "Subsystems/Drive/DriveInfo.h"
#include "Util/RampUtil.h"
#include <memory>

class JackScrewCalculator {
public:
    JackScrewCalculator(std::shared_ptr<SwerveWheel> wheel, int targetDistance, int controlTimeStart) : 
      wheel(wheel), 
      targetDistance(targetDistance),
      controlTimeStart(controlTimeStart) {
        startPosition = currentPosition = lastPosition = wheel->GetDriveEncoderPosition();
    }

    void Run() {
        const double now = frc::Timer::GetFPGATimestamp();
        const double speed = RampUtil::RampUp(controlSpeed, (now - controlTimeStart), jackScrewRampTime, 0.0);

        currentPosition = wheel->GetDriveEncoderPosition();
        lastChange = currentPosition - lastPosition;
        accumulatedPosition += abs(lastChange);
        int remaining = abs(targetDistance - currentPosition);

        if (remaining < rotationCloseLoopThreshold) {
            wheel->UseClosedLoopDrive((startPosition + targetDistance));
        } else {
            wheel->UseOpenLoopDrive(speed);
        }
        lastPosition = currentPosition;
    }

    int GetLastChange() const { return lastChange; }
    int GetAccumulatedPosition() const { return accumulatedPosition; }
    double GetControlSpeed() const { return controlSpeed; }
    void SetControlSpeed(double speed) { controlSpeed = speed; }
    bool IsFinished() const { return abs(currentPosition - targetDistance) <= finishedThreshold; }

private:
    const double controlTimeStart;
    const std::shared_ptr<SwerveWheel> wheel;
    const int targetDistance;
    int startPosition = -1;
    int lastPosition = 0;
    int currentPosition = -1;
    int lastChange = 0;
    int accumulatedPosition;

    double controlSpeed = 1.00;
    const double jackScrewRampTime = 0.25;
    const int rotationCloseLoopThreshold = 50;
    const int finishedThreshold = 5;
};