#include "frc/WPILib.h"
#include "Autonomous/World.h"
#include "frc/DriverStation.h"


World::World() {
	DriverStation::Alliance alliance = DriverStation::GetInstance().GetAlliance();
	isRed = DriverStation::Alliance::kRed == alliance;
	fieldInfo = FieldInfo();
}


void World::Init() {
}

double World::GetClock() const {
	return frc::Timer::GetFPGATimestamp();
}

FieldInfo World::GetFieldInfo() {
	return fieldInfo;
}

void World::SetFieldInfo(FieldInfo _fieldInfo) {
	fieldInfo = _fieldInfo;
}

bool World::IsRed() {
	return isRed;
}


