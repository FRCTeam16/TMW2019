#include "Subsystems/RampSystem.h"
#include "RobotMap.h"
#include <iostream>

RampSystem::RampSystem() {}

void RampSystem::Run() {
    if (!deployed && deployRequested) {
        std::cout << "RampSystem::Run() - Deploying Ramp\n";
        // TODO: Fire solenoids, etc.
        // FIXME: Took solenoid over from intake
        // RobotMap::ejectorSolenoid->Set(true);
        deployed = true;
    }
}

void RampSystem::ToggleDeploy() {
    deployRequested = !deployRequested;
}
