#include "Autonomous/Steps/StopAtTarget.h"
#include "Robot.h"

StopAtTarget::StopAtTarget(Step *step, double _targetArea, int numScans, double _timeOutTime) 
  : step(step), targetArea(_targetArea), numScansToHold(numScans), timeOutTime(_timeOutTime) {}

bool StopAtTarget::Run(std::shared_ptr<World> world) {
    if (startTime < 0) {
        startTime = frc::Timer::GetFPGATimestamp();
    }
    const double elapsed = frc::Timer::GetFPGATimestamp() - startTime;
    timedOut = (elapsed > timeOutTime);
    if (timedOut) {
        std::cout << "StopAtTarget: Stopped due to vision threshold met\n";
        return true;
    }

    const auto vision = Robot::visionSystem->GetLastVisionInfo();
    const bool hasTarget = vision->hasTarget;

    bool visionThresholdMet = false;
    if (targetArea > 0) {
        // Using target area as determiner
        visionThresholdMet = vision->targetArea >= targetArea;
        if (visionThresholdMet) {
            scanCount++;
        } else {
            scanCount = 0;
        }
    } else {
        // Only using hasTarget as determiner
        if (vision->hasTarget) {
            scanCount++; 
        } else {
            scanCount = 0;
        }
    }

    std::cout << "StopAtTarget(scans = " << scanCount << "):" 
                  << " | tgt? " << vision->hasTarget
                  << " | xspeed: " << vision->xSpeed
                  << " | ta: " << vision->targetArea
                  << " | ta met? " << visionThresholdMet
                  << "\n";

    if (scanCount >= numScansToHold) {
        std::cout << "StopAtTarget: Exiting due to vision threshold met\n";
        return true;
    } else {
        return step->Run(world);;
    }
}
