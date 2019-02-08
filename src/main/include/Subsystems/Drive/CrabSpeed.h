#ifndef CRABSPEED_H_
#define CRABSPEED_H_

#include "frc/WPILib.h"

class CrabSpeed : public frc::PIDOutput
{
public:
	CrabSpeed();
	virtual ~CrabSpeed();

	void PIDWrite(double _output) override;
	double Get() const;
	
private:
	double output;
};

#endif
