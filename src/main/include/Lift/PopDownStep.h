#pragma once
#include "Lift/Action.h"
#include "Lift/LiftDrive.h"

class PopDownStep : public Action {
 public:
  PopDownStep() = default;
  void Execute() override;
 private:
  bool liftFinished = false;
};
