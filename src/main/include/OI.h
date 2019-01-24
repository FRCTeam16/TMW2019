// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef OI_H
#define OI_H

#include "frc/WPILib.h"
#include "Util/BSButton.h"

using namespace frc;

class OI {
private:
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	std::shared_ptr<Joystick> gamepad;
	std::shared_ptr<Joystick> driverRight;
	std::shared_ptr<Joystick> driverLeft;

	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	double scaledRadians(double radians);
public:
	OI();

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PROTOTYPES

	std::shared_ptr<Joystick> getDriverLeft();
	std::shared_ptr<Joystick> getDriverRight();
	std::shared_ptr<Joystick> getGamepad();

	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PROTOTYPES
	std::shared_ptr<BSButton> GPX;
	std::shared_ptr<BSButton> GPY;
	std::shared_ptr<BSButton> GPB;
	std::shared_ptr<BSButton> GPA;
	std::shared_ptr<BSButton> GPLB;
	std::shared_ptr<BSButton> GPRB;
	std::shared_ptr<BSButton> GPBack;
	std::shared_ptr<BSButton> GPStart;
	std::shared_ptr<BSButton> DL1;
	std::shared_ptr<BSButton> DL2;
	std::shared_ptr<BSButton> DL3;
	std::shared_ptr<BSButton> DL4;
	std::shared_ptr<BSButton> DL5;
	std::shared_ptr<BSButton> DL6;
	std::shared_ptr<BSButton> DL7;
	std::shared_ptr<BSButton> DL8;
	std::shared_ptr<BSButton> DL9;
	std::shared_ptr<BSButton> DL10;
	std::shared_ptr<BSButton> DL11;
	std::shared_ptr<BSButton> DL12;
	std::shared_ptr<BSButton> DR1;
	std::shared_ptr<BSButton> DR2;
	std::shared_ptr<BSButton> DR3;
	std::shared_ptr<BSButton> DR4;
	std::shared_ptr<BSButton> DR5;
	std::shared_ptr<BSButton> DR6;
	std::shared_ptr<BSButton> DR7;
	std::shared_ptr<BSButton> DR8;
	std::shared_ptr<BSButton> DR9;
	std::shared_ptr<BSButton> DR10;
	std::shared_ptr<BSButton> DR11;
	std::shared_ptr<BSButton> DR12;

	enum DPad {
		kUnpressed = -1,
		kUp = 0,
		kUpRight = 45,
		kRight = 90,
		kDownRight = 135,
		kDown = 180,
		kDownLeft = 225,
		kLeft = 270,
		kUpLeft = 315,
		kUnknown = 999
	};

	DPad GetGamepadDPad();
	double GetJoystickTwist(double threshold = 0.1);
	double GetJoystickX(double threshold = 0.1);
	double GetJoystickY(double threshold = 0.1);
	double GetGamepadLeftStick();
	double GetGamepadRightStick();
	double GetGamepadLT();
	double GetGamepadRT();
	void SetGamepadLeftRumble(double rumble);
	void SetGamepadRightRumble(double rumble);
	void SetGamepadBothRumble(double rumble);

	double getLeftJoystickXRadians();
	double getScaledJoystickRadians();
};

#endif
