#ifndef SRC_UTIL_DISTANCECONTROL_H_
#define SRC_UTIL_DISTANCECONTROL_H_

#include "frc/WPILib.h"

using namespace frc;


/**
 * Used to provide steering output required to maintain a target distance
 */
class DistanceControl : public PIDOutput {
public:
	explicit DistanceControl(std::shared_ptr<AnalogInput> _ultrasonic, double _target = 0);
	virtual ~DistanceControl();
	void SetTarget(double _target);
	void PIDWrite (double output) override;
	void SetOutputRange(double _min, double _max);
	void SetInvert(bool _invert);

	void Enable();
	void Disable();

	double Get() const;
private:
	const std::shared_ptr<AnalogInput> ultrasonic;
	std::unique_ptr<PIDController> pid;
	double invert = 1.0;
	double output = 0.0;
};

#endif /* SRC_UTIL_DISTANCECONTROL_H_ */
