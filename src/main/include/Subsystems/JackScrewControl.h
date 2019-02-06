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
    explicit JackScrewControl(std::shared_ptr<SwerveWheel> wheel) : wheel(wheel) {
        std::cout << "********************* JackScrewControl *********************\n";
        currentState = JackScrewState::kSwerve;
    }
    enum class JackScrewState { kSwerve, kOpenLoop, kClosedLoop };

    void Init(double targetDistance, double controlTimeStart);
    void InitOpenLoop(double speed);
    void Run();
    void Hold();

    double GetLastChange() { return lastChange; }
    double GetAccumulatedPosition() { return accumulatedPosition; }
    double GetControlSpeed() { return controlSpeed; }
    void SetControlSpeed(double speed) { controlSpeed = speed; }
    //bool IsFinished() const { return abs(currentPosition - targetDistance) <= finishedThreshold; }
    bool IsClosedLoop() { return currentState == JackScrewState::kClosedLoop; }
    double GetTargetDistance() { return startPosition + targetDistance; }
    JackScrewState GetCurrentState() { return currentState; }
    void SetCurrentState(JackScrewState newState) { 
        std::cout << "Setting JackScrewControl " << this << " to state " << static_cast<int>(newState) << "\n";
        currentState = newState; 
    }
    void SetAutoSwitchToControl(bool _value) { autoSwitchToControl = _value; }

private:
    const std::shared_ptr<SwerveWheel> wheel;
    JackScrewState currentState;
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
    bool autoSwitchToControl = false;
};