#ifndef SRC_AUTONOMOUS_STEPS_POSITIONELEVATOR_H_
#define SRC_AUTONOMOUS_STEPS_POSITIONELEVATOR_H_

#include "Autonomous/Step.h"
#include "Subsystems/Elevator.h"
#include "Autonomous/Steps/DelayParam.h"

class PositionElevator: public Step {
public:
	PositionElevator(Elevator::ElevatorPosition elevatorPosition, DelayParam delayParam, bool _waitForPosition = false, double _timeout=3.0) :
		position(elevatorPosition), delayParam(delayParam), waitForPosition(_waitForPosition), timeout(_timeout) {}

	PositionElevator(Elevator::ElevatorPosition elevatorPosition, bool _waitForPosition = false, double _timeout=3.0) :
		position(elevatorPosition), waitForPosition(_waitForPosition), timeout(_timeout) {}

	virtual ~PositionElevator() {}
	bool Run(std::shared_ptr<World> world) override;

	void SetOverrideElevatorPosition(int _pos) { overrideElevatorPosition = _pos; }

private:
	const Elevator::ElevatorPosition position;
	DelayParam delayParam;
	bool firstRun = true;
	bool sentPosition = false;
	double target = 0;	// tracks position or time if a delay was requeested
	bool waitForPosition = false;
	const double timeout;

	int overrideElevatorPosition = -1;

	void DoSetPosition();

};

#endif /* SRC_AUTONOMOUS_STEPS_POSITIONELEVATOR_H_ */
