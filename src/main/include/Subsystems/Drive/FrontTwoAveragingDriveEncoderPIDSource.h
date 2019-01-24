/*
 * AveragingDriveEncoderPIDSource.h
 *
 *  Created on: Mar 26, 2017
 *      Author: User
 */

#ifndef SRC_SUBSYSTEMS_DRIVE_FRONTTWOAVERAGINGDRIVEENCODERPIDSOURCE_H_
#define SRC_SUBSYSTEMS_DRIVE_FRONTTWOAVERAGINGDRIVEENCODERPIDSOURCE_H_

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "Subsystems/Drive/DriveInfo.h"
#include "SwerveWheel.h"

class FrontTwoAveragingDriveEncoderPIDSource : public frc::PIDSource {
public:
	FrontTwoAveragingDriveEncoderPIDSource(DriveInfo<std::shared_ptr<SwerveWheel>> _wheels);
	virtual ~FrontTwoAveragingDriveEncoderPIDSource();
	virtual double PIDGet();
	void SetInitialEncoderValue();
	void SetShowDebug(bool _showDebug);
private:
	DriveInfo<std::shared_ptr<SwerveWheel>> wheels;
	DriveInfo<double> initialEncoderValue;
	double CalculateAverage(const DriveInfo<double> &error, const DriveInfo<bool> &motorEnabled);
	bool showDebug = false;
};

#endif /* SRC_SUBSYSTEMS_DRIVE_FRONTTWOAVERAGINGDRIVEENCODERPIDSOURCE_H_ */
