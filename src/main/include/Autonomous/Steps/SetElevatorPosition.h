#pragma once
#include "Autonomous/Step.h"
#include "Subsystems/Elevator.h"

class SetElevatorPosition : public Step {
public:
  SetElevatorPosition(Elevator::ElevatorPosition targetPosition, double timeOut);
  bool Run(std::shared_ptr<World> world) override;
private:
  const Elevator::ElevatorPosition position;
  const double timeOut;
  double startTime = -1;
};
