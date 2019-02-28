#ifndef SRC_AUTONOMOUS_STEPS_TIMEDDRIVE_H_
#define SRC_AUTONOMOUS_STEPS_TIMEDDRIVE_H_

#include "frc/WPILib.h"
#include "Autonomous/Step.h"

class TimedDrive: public Step {
public:
	TimedDrive(double _angle, double y, double x, double driveTime, double rampUpTime = -1, bool _useTwist = true) :
			angle(_angle),
			ySpeed(y),
			xSpeed(x),
			timeToDrive(driveTime),
			rampUpTime(rampUpTime),
			useTwist(_useTwist) {}
	virtual ~TimedDrive() {}
	bool Run(std::shared_ptr<World> world) override;

private:
    const double angle;
    const double ySpeed;
    const double xSpeed;
    const double timeToDrive;
	const double rampUpTime;
    const bool useTwist;
    double startTime = -1;
};

#endif /* SRC_AUTONOMOUS_STEPS_TIMEDDRIVE_H_ */
