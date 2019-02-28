#pragma once

#include "Autonomous/Step.h"
#include "Subsystems/IntakeRotate.h"

class RotateIntake : public Step {
public:
  RotateIntake(IntakeRotate::IntakePosition _target, bool waitForPosition = false, double timeout = 0.0);
  bool Run(std::shared_ptr<World> world) override;
private:
  const IntakeRotate::IntakePosition targetPosition;
  const bool waitForPosition;
  const double timeout;
  double startTime = -1;
};
