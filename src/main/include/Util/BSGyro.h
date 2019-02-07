/*
 * BSGyro.h
 *
 *  Created on: Feb 27, 2017
 *      Author: User
 */

#ifndef SRC_UTIL_BSGYRO_H_
#define SRC_UTIL_BSGYRO_H_

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

using namespace frc;

class BSGyro : public frc::PIDSource {
private:
	std::unique_ptr<PigeonIMU> pigeon;
	float GetOffset();
	float offset = 0.0;
public:
    explicit BSGyro(WPI_TalonSRX *talon);

	explicit BSGyro(int canId);
	virtual ~BSGyro();
	double PIDGet();
	PigeonIMU* GetPigeon();
	void SetOffset(float offset);

    float GetYaw();
    void ZeroYaw();

	double ReadYaw();

};

#endif /* SRC_UTIL_BSGYRO_H_ */
