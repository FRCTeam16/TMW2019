#pragma once

#include "Autonomous/Step.h"

class AlignToTarget : public Step {
 public:
  AlignToTarget(double _angle, double _target, double _timeout);
  bool Run(std::shared_ptr<World> world) override;
 private:
  const double angle;
  const double target;
  const double timeout;
  double startTime = -1;
};
