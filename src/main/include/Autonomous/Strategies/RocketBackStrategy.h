#pragma once
#include "Autonomous/Strategy.h"

class RocketBackStrategy : public StepStrategy {
 public:
  explicit RocketBackStrategy(std::shared_ptr<World> worldh) {};
  void Init(std::shared_ptr<World> world) override;
};
