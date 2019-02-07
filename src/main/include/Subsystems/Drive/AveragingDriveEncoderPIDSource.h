/*
 * AveragingDriveEncoderPIDSource.h
 *
 *  Created on: Mar 26, 2017
 *      Author: User
 */

#ifndef SRC_SUBSYSTEMS_DRIVE_AVERAGINGDRIVEENCODERPIDSOURCE_H_
#define SRC_SUBSYSTEMS_DRIVE_AVERAGINGDRIVEENCODERPIDSOURCE_H_

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "Subsystems/Drive/DriveInfo.h"


class AveragingDriveEncoderPIDSource : public frc::PIDSource {
public:
    explicit AveragingDriveEncoderPIDSource(DriveInfo<std::shared_ptr<WPI_TalonSRX>> _motor);

	~AveragingDriveEncoderPIDSource() override;

	double PIDGet() override;
	void SetInitialEncoderValue();
	void SetShowDebug(bool _showDebug);
private:
	DriveInfo<std::shared_ptr<WPI_TalonSRX>> motor;
	DriveInfo<double> initialEncoderValue;
	double CalculateAverage(const DriveInfo<double> &error, const DriveInfo<bool> &motorEnabled);
	bool showDebug = false;
};

#endif /* SRC_SUBSYSTEMS_DRIVE_AVERAGINGDRIVEENCODERPIDSOURCE_H_ */
