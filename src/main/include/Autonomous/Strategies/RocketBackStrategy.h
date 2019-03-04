#pragma once
#include "Autonomous/Strategy.h"

class RocketBackStrategy : public StepStrategy {
 public:
  explicit RocketBackStrategy() = default;
  void Init(std::shared_ptr<World> world) override;
};
