#include "Autonomous/Steps/SetElevatorPosition.h"

SetElevatorPosition::SetElevatorPosition(Elevator::ElevatorPosition targetPosition) 
  : position(targetPosition) {}


bool SetElevatorPosition::Run(std::shared_ptr<World> world) {
    
}