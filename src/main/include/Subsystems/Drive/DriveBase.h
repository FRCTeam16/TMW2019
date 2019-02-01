// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef DRIVEBASE_H
#define DRIVEBASE_H

#include "rev/CANSparkMax.h"
#include "frc/commands/Subsystem.h"
#include "frc/WPILib.h"
#include "CrabSpeed.h"
#include "DriveEncoderPIDSource.h"
#include "FrontTwoAveragingDriveEncoderPIDSource.h"
#include "DriveInfo.h"
#include "SwerveWheel.h"

using namespace frc;


struct Wheelbase {
	double W = 0;
	double X = 0;
	double Y = 0;
};

class DriveBase: public Subsystem {
private:

	std::shared_ptr<SwerveWheel> frontLeft;
	std::shared_ptr<SwerveWheel> frontRight;
	std::shared_ptr<SwerveWheel> rearLeft;
	std::shared_ptr<SwerveWheel> rearRight;
	

	std::unique_ptr<CrabSpeed> crabSpeedTwist;
	std::unique_ptr<FrontTwoAveragingDriveEncoderPIDSource> driveControlEncoderSource;



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

	DriveInfo<double> positionOffsets;
	DriveInfo<int> turns;
	DriveInfo<int> inv;
	DriveInfo<int> hotCount;

	double A = 0;					// steer mode
	double thetaRC = 0;
	DriveInfo<double> theta;		// steer mode theta
	DriveInfo<double> steerSpeed;	// steer mode wheel speeds

	int coolCount = 0;
	double driveLimit = 1.0;

	Wheelbase wheelbase;
	std::unique_ptr<PIDController> driveControlSpeedController;
	std::unique_ptr<PIDController> driveControlTwist;
	std::unique_ptr<CrabSpeed> driveControlDistanceSpeed;

	const int MaxTurns = 1000;
	bool driveFront = true;

	void InitializePIDs();
	void InitializeOffsets();

	void SetSteerSetpoint(double setpoint,
			std::shared_ptr<AnalogInput> actual, double offset,
			std::shared_ptr<PIDController> PIDCon, std::shared_ptr<WPI_TalonSRX> steer,
			int turns, int &inv);

	//created for new talon positioning
	double GetTalonCurrentPosition(std::shared_ptr<WPI_TalonSRX> talon);
	int GetTalonCurrentTurns(std::shared_ptr<WPI_TalonSRX> talon);
	double GetTalonCurrentOffsetPosition(std::shared_ptr<WPI_TalonSRX> talon);


	double CorrectSteerSetpoint(double setpoint);
	void SetDriveSpeed(DriveInfo<double> speed);

	bool lastSpeedX = 0.0;	// last speed used for DMS
	bool lastSpeedY = 0.0;

public:
	DriveBase();
	void InitDefaultCommand();
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
	DriveInfo<double> CalculatePositionOffsets();
	void SetPositionOffsets(DriveInfo<double> &info);
	DriveInfo<double> GetPositionOffsets() const;
	void ZeroTurnInfo();
	void ZeroDriveEncoders();
	void SetSteering(DriveInfo<double> setpoint);

	void InitTeleop();
	void InitAuto();

	void Lock();
	void Crab(double twist, double y, double x, bool useGyro);

	void SetTargetAngle(double angle);
	double GetTwistControlOutput();
	double GetTwistControlError();
	double GetCrabTwistOutput();

	void SetTargetDriveDistance(double distance, double maxSpeed = 0.5);
	void UseClosedLoopDrive();
	void UseOpenLoopDrive();

	double GetDriveControlEncoderPosition();
	double GetDriveControlOutput();
	double GetDriveControlError();
	double GetDriveControlP();
	double GetDriveControlSetpoint();

	Wheelbase GetWheelbase();

	void SetConstantVelocity(double twistInput, double velocity);

	// FIXME: Should be private
	std::vector<std::shared_ptr<SwerveWheel>> wheels;

	const double GetLastSpeedX() { return lastSpeedX; }
	const double GetLastSpeedY() { return lastSpeedY; }
	void Instrument();
	void Diagnostics();	// used for more detailed diagnostics displayed on smart dashboard

	DriveInfo<double> GetDriveEncoderPositions();
	DriveInfo<double> GetDriveCurrent();
	DriveInfo<int> GetDMSDriveVelocity();
	DriveInfo<int> GetSteerEncoderPositions();	// FIXME?
	DriveInfo<double> GetSteerCurrent();
	DriveInfo<int> GetDMSSteerVelocity();

	void DMSDrive(double speed);
	void DMSSteer(double speed);
};
#endif
