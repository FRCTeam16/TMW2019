#include "Autonomous/Steps/SetElevatorPosition.h"
#include <iostream>
#include "Robot.h"

SetElevatorPosition::SetElevatorPosition(Elevator::ElevatorPosition targetPosition, double timeOut) 
  : position(targetPosition), timeOut(timeOut) {}


bool SetElevatorPosition::Run(std::shared_ptr<World> world) {
    if (startTime < 0) {
      startTime = frc::Timer::GetFPGATimestamp();
      Robot::elevator->SetElevatorPosition(position);
    }
    const double elapsed = frc::Timer::GetFPGATimestamp() - startTime;
    if ((timeOut > 0) && (elapsed > timeOut)) {
      std::cout << "*** SetElevatorPosition timed out at: " << elapsed << "\n";
      return true;
    }

    return Robot::elevator->InPosition();
}