#include "Autonomous/AutoManager.h"
#include "Autonomous/AutoPositions.h"
#include "RobotMap.h"
#include "Autonomous/Strategies/DebugAutoStrategy.h"
#include "Robot.h"



AutoManager::AutoManager() :
		positions(new frc::SendableChooser<int>()),
		strategies(new frc::SendableChooser<int>())
{
	// strategies->AddDefault("1 - Side Start", AutoStrategy::kSide);
	strategies->SetDefaultOption("99 - Debug Auto Strategy", AutoStrategy::kDebug);

	positions->SetDefaultOption("2 - Right", AutoStartPosition::kRight);
	positions->AddOption("1 - Center", AutoStartPosition::kCenter);
	positions->AddOption("0 - Left",  AutoStartPosition::kLeft);

	frc::SmartDashboard::PutData("Autonomous Start Pos0", positions.get());
	frc::SmartDashboard::PutData("Autonomous Strategy1", strategies.get());
	std::cout << "AutoManager::AutoManager() finished\n";
}


std::unique_ptr<Strategy> AutoManager::CreateStrategy(const AutoStrategy &key, std::shared_ptr<World> world) {
	const frc::DriverStation::Alliance alliance = frc::DriverStation::GetInstance().GetAlliance();
	const bool isRed =  alliance == frc::DriverStation::Alliance::kRed;
	std::cout << "AutoManager::CreateStrategy -> isRed = " << isRed << "\n";

	Strategy *strategy = 0;
	switch (key) {
	case kDebug:
		std::cout << "AUTOMAN: Selected DEBUG \n";
		strategy = new DebugAutoStrategy(world);
		break;
	default:
		// TODO: Fill in sane default
		std::cerr << "No valid strategy selected\n";
	}
	return std::unique_ptr<Strategy>(strategy);
}


void AutoManager::Init(std::shared_ptr<World> world) {
	std::cout << "AutoMan Init\n";

	const AutoStartPosition selectedPosition = static_cast<AutoStartPosition>(positions->GetSelected());
	std::cout << "AutoMan Position selectedKey: " << selectedPosition << "\n";
	world->SetStartPosition(selectedPosition);

	const AutoStrategy selectedKey = static_cast<AutoStrategy>(strategies->GetSelected());
	std::cout << "AutoMan Init selectedKey: " << selectedKey << "\n";

	currentStrategy = CreateStrategy(selectedKey, world);
	if (!currentStrategy) {
		std::cerr << "NO AUTONOMOUS STRATEGY FOUND\n";
	}

	RobotMap::gyro->ZeroYaw();
	currentStrategy->Init(world);

	startTime = -1;
	finalPhaseFired = false;
	std::cout << "AutoManager::Init COMPLETE\n";
}

void AutoManager::Periodic(std::shared_ptr<World> world) {
	const double currentTime = world->GetClock();
	if (currentStrategy) {

        // Perform global startup runtime system actions here
		if (startTime < 0) {
			startTime = currentTime;
		}

        // Perform any global post-init runtime system actions here
		if ((currentTime - startTime) > 1) {
		}

        // Perform any global final system actions here
		if (((currentTime - startTime) > 13) && !finalPhaseFired) {
			finalPhaseFired = true;

		}
		currentStrategy->Run(world);
	}
}

void AutoManager::Instrument() {
	const AutoStrategy selectedKey = static_cast<AutoStrategy>(strategies->GetSelected());
	frc::SmartDashboard::PutNumber("Selected Auto", selectedKey);

	const AutoStartPosition selectedPosition = static_cast<AutoStartPosition>(positions->GetSelected());
	frc::SmartDashboard::PutNumber("Auto Selected Position", selectedPosition);
}
