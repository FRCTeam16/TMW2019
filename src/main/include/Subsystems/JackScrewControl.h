#pragma once

#include "frc/Preferences.h"
#include "frc/Timer.h"
#include "Subsystems/Drive/SwerveWheel.h"
#include "Subsystems/Drive/DriveInfo.h"
#include "Util/RampUtil.h"
#include <memory>
#include <iostream>

class JackScrewControl {
public:
    JackScrewControl(std::shared_ptr<SwerveWheel> wheel) : wheel(wheel) {
        currentState = JackScrewState::kSwerve;
    }
    enum class JackScrewState { kSwerve, kOpenLoop, kClosedLoop };

    void Init(int targetDistance, int controlTimeStart);
    void Run();
    void Hold();

    double GetLastChange() const { return lastChange; }
    double GetAccumulatedPosition() const { return accumulatedPosition; }
    double GetControlSpeed() const { return controlSpeed; }
    void SetControlSpeed(double speed) { controlSpeed = speed; }
    //bool IsFinished() const { return abs(currentPosition - targetDistance) <= finishedThreshold; }
    bool IsClosedLoop() const { return currentState == JackScrewState::kClosedLoop; }
    double GetTargetDistance() const { return startPosition + targetDistance; }
    JackScrewState GetCurrentState() const { return currentState; }
    void SetCurrentState(JackScrewState newState) { currentState = newState; }

private:
    const std::shared_ptr<SwerveWheel> wheel;
    JackScrewState currentState = JackScrewState::kSwerve;
    double controlTimeStart = -1;
    double targetDistance = 0;
    double startPosition = 0;
    double lastPosition = 0;
    double currentPosition = 0;
    double lastChange = 0;
    double accumulatedPosition = 0;

    double controlSpeed = 1.0;  // FIXME: Duplicated in jackscrews
    const double jackScrewRampTime = 0.1;
    const double rotationCloseLoopThreshold = 7;
    const double finishedThreshold = 1;
};