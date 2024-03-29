#include "Autonomous/AutoManager.h"
#include "Autonomous/AutoPositions.h"
#include "RobotMap.h"
#include "Robot.h"

#include "Autonomous/Strategies/DebugAutoStrategy.h"
#include "Autonomous/Strategies/TwoHatchCenterStartStrategy.h"
#include "Autonomous/Strategies/RocketBackStrategy.h"
#include "Autonomous/Strategies/MultiCenterStrategy.h"



AutoManager::AutoManager() :
		positions(new frc::SendableChooser<int>()),
		strategies(new frc::SendableChooser<int>())
{
	strategies->AddOption("0 - None", AutoStrategy::kNone);
	strategies->SetDefaultOption("1 - TwoHatchCenterStartStrategy", AutoStrategy::kTwoHatchCenterStartStrategy);
	strategies->AddOption("2 - Rocket Back Strategy", AutoStrategy::kRocketBackStrategy);
	strategies->AddOption("3 - Multi-Cargo Ship Strategy", AutoStrategy::kMultiCargoShipStrategy);
	strategies->AddOption("99 - Debug Auto Strategy", AutoStrategy::kDebug);

	positions->SetDefaultOption("2 - Right", AutoStartPosition::kRight);
	// positions->AddOption("1 - Center", AutoStartPosition::kCenter);
	positions->AddOption("0 - Left",  AutoStartPosition::kLeft);

	frc::SmartDashboard::PutData("Autonomous Start Pos - DET", positions.get());
	frc::SmartDashboard::PutData("Autonomous Strategy - DET", strategies.get());
	std::cout << "AutoManager::AutoManager() finished\n";
}


std::unique_ptr<Strategy> AutoManager::CreateStrategy(const AutoStrategy &key, std::shared_ptr<World> world) {
	const frc::DriverStation::Alliance alliance = frc::DriverStation::GetInstance().GetAlliance();
	const bool isRed =  alliance == frc::DriverStation::Alliance::kRed;
	std::cout << "AutoManager::CreateStrategy -> isRed = " << isRed << "\n";

	Strategy *strategy = 0;
	switch (key) {
	case kNone:
		std::cout << "AUTOMAN: Selected NONE \n";
		strategy = new StepStrategy();
		break;
	case kDebug:
		std::cout << "AUTOMAN: Selected DEBUG \n";
		strategy = new DebugAutoStrategy(world);
		break;
	case kTwoHatchCenterStartStrategy:
		std::cout << "AUTOMAN: Selected kTwoHatchCenterStartStrategy \n";
		strategy = new TwoHatchCenterStartStrategy(world);
		break;
	case kRocketBackStrategy:
		std::cout << "AUTOMAN: Selected kRocketBackStrategy \n";
		strategy = new RocketBackStrategy(world);
		break;
	case kMultiCargoShipStrategy:
		std::cout << "AUTOMAN: Selected kMultiCargoShipStrategy \n";
		strategy = new MultiCenterStrategy(world);
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

	// Removed 2019 since we can run auto multiple times: RobotMap::gyro->ZeroYaw();
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
	frc::SmartDashboard::PutNumber("Selected Auto Strat: ", selectedKey);

	const AutoStartPosition selectedPosition = static_cast<AutoStartPosition>(positions->GetSelected());
	frc::SmartDashboard::PutNumber("Selected Auto Position: ", selectedPosition);
}
