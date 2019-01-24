#include <iostream>
#include "Commands/ZeroGyroOffset.h"
#include "RobotMap.h"


ZeroGyroOffset::ZeroGyroOffset() : Command() {
	SetRunWhenDisabled(true);
}

void ZeroGyroOffset::Initialize() {
	std::cout << "****** Zeroing BSGyro Offset ******\n";
	RobotMap::gyro->SetOffset(0.0);
	SetTimeout(1);
}

void ZeroGyroOffset::Execute() {
}

bool ZeroGyroOffset::IsFinished() {
	return IsTimedOut();
}

void ZeroGyroOffset::End() {
}

void ZeroGyroOffset::Interrupted() {

}
