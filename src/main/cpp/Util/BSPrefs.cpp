#include "Util/BSPrefs.h"
#include <iostream>
#include "Util/BSPrefsFilebased.h"

BSPrefs * BSPrefs::instance;

BSPrefs* BSPrefs::GetInstance() {
    if (instance == nullptr) {
        instance = new BSPrefsFilebased(); 
        instance->LoadConstants();       
    }
    return instance;
}

void BSPrefs::LoadConstants() {
    std::cout << "Loading constants into BSPrefs\n";
    constants = RAWCConstants::getInstance();

    for (auto const& kv : constants->getConstants()) {
        lookupDouble[kv.first] = kv.second;
    }
    std::cout << "BSPrefs completed construction\n";
}

double BSPrefs::GetDouble(std::string key, double value) const {
    if (lookupDouble.count(key) > 0) {
        return lookupDouble.at(key);
    } else {
        std::cout << "!!! BSPrefs::lookupDouble Missing Key: " << key << " | " << value << "\n";
        return value;
    }
}

int BSPrefs::GetInt(std::string key, int value) const {
    return static_cast<int>(GetDouble(key, value));
}

bool BSPrefs::GetBool(std::string key, bool value) const {
    if (lookupBool.count(key) > 0) {
        return lookupBool.at(key);
    } else {
        std::cout << "!!! BSPrefs::lookupBool Missing Key: " << key << " | " << value << "\n";
        return value;
    }
}


void BSPrefs::StoreDouble(std::string key, double value) {
    constants->insertKeyAndValue(key, value);
    lookupDouble[key] = value;
}

void BSPrefs::SaveConstants() {
    std::cout << "BSPrefs : Saving Constants Started\n";
    constants->save();
    std::cout << "BSPrefs : Saving Constants Finished\n";
}


/************************************************************************/

BSPrefsHardcoded::BSPrefsHardcoded() {
    // Begin autogenerated code
    // UNKNOWN DATATYPE [string].emplace(".type", "RobotPreferences");
    lookupDouble.emplace("Auto.MCS.PushBack.y", -0.15);
    lookupDouble.emplace("Auto.MCS.cargoDrive1.time", 1.75);
    lookupDouble.emplace("Auto.MCS.cargoDrive1.x", 0.2);
    lookupDouble.emplace("Auto.MCS.cargoDrive1.y", 0.4);
    lookupDouble.emplace("Auto.MCS.cargoDrive2.ignoretime", 0);
    lookupDouble.emplace("Auto.MCS.cargoDrive2.tgt", 3);
    lookupDouble.emplace("Auto.MCS.cargoDrive2.time", 5);
    lookupDouble.emplace("Auto.MCS.cargoDrive2.y", 0.3);
    lookupDouble.emplace("Auto.MCS.cargoDrive3.time", 1.5);
    lookupDouble.emplace("Auto.MCS.cargoDrive3.x", -0.3);
    lookupDouble.emplace("Auto.MCS.cargoDrive3.y", 0.5);
    lookupDouble.emplace("Auto.MCS.cargoDrive4.ignoretime", 0);
    lookupDouble.emplace("Auto.MCS.cargoDrive4.tgt", 2);
    lookupDouble.emplace("Auto.MCS.cargoDrive4.time", 5);
    lookupDouble.emplace("Auto.MCS.cargoDrive4.y", 0.3);
    lookupDouble.emplace("Auto.MCS.cargoDriveTimeout", 3.5);
    lookupDouble.emplace("Auto.MCS.initDriveSpeed.x", 0);
    lookupDouble.emplace("Auto.MCS.initDriveSpeed.y", 0.3);
    lookupDouble.emplace("Auto.MCS.pickupFastSpeed.x", 0.2);
    lookupDouble.emplace("Auto.MCS.pickupFastSpeed.y", -0.7);
    lookupDouble.emplace("Auto.MCS.pickupFastSpeedRamp", 0.5);
    lookupDouble.emplace("Auto.MCS.pickupFastTime1", 2.5);
    lookupDouble.emplace("Auto.MCS.pickupFastTime2", 1);
    lookupDouble.emplace("Auto.MCS.pickupVizSpeed", 0.25);
    lookupDouble.emplace("Auto.MCS.pickupVizTime", 5);
    lookupDouble.emplace("Auto.MCS.pickupVizTimeout", 2.5);
    lookupDouble.emplace("Auto.MCS.placeHatchThreshold", 5);
    lookupDouble.emplace("Auto.MCS.placeHatchY", 0.3);

    lookupDouble.emplace("Auto.RBS.initDriveSpeed", 0.3);
    lookupDouble.emplace("Auto.RBS.pickupFastSpeed", -0.4);
    lookupDouble.emplace("Auto.RBS.pickupFastSpeedX", 0.1);
    lookupDouble.emplace("Auto.RBS.pickupFastTime1", 2);
    lookupDouble.emplace("Auto.RBS.pickupFastTime2", 1);
    lookupDouble.emplace("Auto.RBS.pickupVizSpeed", 0.25);
    lookupDouble.emplace("Auto.RBS.pickupVizTime", 5);
    lookupDouble.emplace("Auto.RBS.pickupVizTimeout", 2.5);
    lookupDouble.emplace("Auto.RBS.pushBackSpeed", 0.15);
    lookupDouble.emplace("Auto.RBS.rocketAlignmentScans", 4);
    lookupDouble.emplace("Auto.RBS.rocketAlignmentThreshold", 0.3);
    lookupDouble.emplace("Auto.RBS.rocketAlignmentTime", 3);
    lookupDouble.emplace("Auto.RBS.rocketAngle", 150);
    lookupDouble.emplace("Auto.RBS.rocketDistThresh", 5);
    lookupDouble.emplace("Auto.RBS.rocketDistance", 160);
    lookupDouble.emplace("Auto.RBS.rocketKickoutSpeed", -0.2);
    lookupDouble.emplace("Auto.RBS.rocketKickoutTime", 0.75);
    lookupDouble.emplace("Auto.RBS.rocketPushInTime", 1.5);
    lookupDouble.emplace("Auto.RBS.rocketPushOutTime", 0.5);
    lookupDouble.emplace("Auto.RBS.rocketRampDownDist", 12);
    lookupDouble.emplace("Auto.RBS.rocketRampUpTime", 0.5);
    lookupDouble.emplace("Auto.RBS.rocketSpeedX", 0.21);
    lookupDouble.emplace("Auto.RBS.rocketSpeedY", 0.45);
    lookupDouble.emplace("Auto.RBS.rocketTimeOut", 3);
    lookupDouble.emplace("Auto.RBS.secRocketAngle", 30);
    lookupDouble.emplace("Auto.RBS.secRocketDriveTargetY", 0.2);
    lookupDouble.emplace("Auto.RBS.secRocketDriveTimeout", 0.2);
    lookupDouble.emplace("Auto.RBS.secRocketRamp", 1);
    lookupDouble.emplace("Auto.RBS.secRocketTimeout", 8);
    lookupDouble.emplace("Auto.RBS.secRocketVizBlindTime", 1);
    lookupDouble.emplace("Auto.RBS.secRocketVizMax", 8);
    lookupDouble.emplace("Auto.RBS.secRocketVizMin", 5);
    lookupDouble.emplace("Auto.RBS.secRocketVizThresh", 5);
    lookupDouble.emplace("Auto.RBS.secRocketX", -0.07);
    lookupDouble.emplace("Auto.RBS.secRocketY", 0.4);

    lookupDouble.emplace("Auto.THCS.D1.time", 1.5);
    lookupDouble.emplace("Auto.THCS.D1.y", 0.3);
    lookupDouble.emplace("Auto.THCS.D2.ignoreTime", 2);
    lookupDouble.emplace("Auto.THCS.D2.time", 1.5);
    lookupDouble.emplace("Auto.THCS.D2.x", 0.5);
    lookupDouble.emplace("Auto.THCS.D2.y", -0.168);
    lookupDouble.emplace("Auto.THCS.D3.y", 0.25);
    lookupDouble.emplace("Auto.THCS.DriveToTarget.MTA", 8);
    lookupDouble.emplace("Auto.THCS.DriveToTarget.TA", 5);
    lookupDouble.emplace("Auto.THCS.PushBack.y", -0.15);
    lookupDouble.emplace("Auto.THCS.cargoDriveThreshold", 5);
    lookupDouble.emplace("Auto.THCS.cargoDriveTimeout", 3.5);
    lookupDouble.emplace("Auto.THCS.cargoDriveY", 0.3);
    lookupDouble.emplace("Auto.THCS.nearCSDriveTime", 2.25);
    lookupDouble.emplace("Auto.THCS.nearCSX", -0.186);
    lookupDouble.emplace("Auto.THCS.nearCSY", 0.6);
    lookupDouble.emplace("Auto.THCS.toCargoIgnoreTime", 0.5);
    lookupDouble.emplace("Auto.THCS.toCargoTime", 2.75);
    lookupDouble.emplace("Auto.THCS.toCargoVizThresh", 10);
    lookupDouble.emplace("Auto.THCS.toCargoX", -0.065);
    lookupDouble.emplace("Auto.THCS.toCargoY", 0.2);

    lookupDouble.emplace("DriveControlP", 0);
    lookupDouble.emplace("DriveControlI", 0);
    lookupDouble.emplace("DriveControlD", 0);
    lookupDouble.emplace("DriveControlF", 0);
    lookupDouble.emplace("DriveControlIZone", 0);
    lookupDouble.emplace("DriveControlTwistIZone", 0);

    lookupDouble.emplace("DriveD", 0);
    lookupDouble.emplace("DriveF", 0);
    lookupDouble.emplace("DriveI", 0);
    lookupDouble.emplace("DriveIZone", 0);
    lookupDouble.emplace("DriveP", 0.4);

    lookupDouble.emplace("Elevator.A", 10000);
    lookupDouble.emplace("Elevator.BumpUp", 0);
    lookupDouble.emplace("Elevator.F", 0.25);
    lookupDouble.emplace("Elevator.I", 0);
    lookupDouble.emplace("Elevator.NV.L1", -30000);
    lookupDouble.emplace("Elevator.NV.L2", -79000);
    lookupDouble.emplace("Elevator.NV.L3", -101000);
    lookupDouble.emplace("Elevator.P", 1);
    lookupDouble.emplace("Elevator.Pos.MaxHeight", -110000);
    lookupDouble.emplace("Elevator.Pos.Threshold", 10);
    lookupDouble.emplace("Elevator.V", 12000);
    lookupDouble.emplace("Elevator.V.L2", -46300);
    lookupDouble.emplace("Elevator.V.L3", -93000);
    lookupDouble.emplace("Elevator.cargoship.shot", -62500);
    lookupDouble.emplace("Elevator.izone", 0);
    lookupDouble.emplace("Elevator.pos.Floor", 0);

    lookupDouble.emplace("Intake.EjectCargo.bottomSpeed", -0.4);
    lookupDouble.emplace("Intake.EjectCargo.topSpeed", -1);
    lookupDouble.emplace("Intake.EjectHatch.bottomSpeed", 1);
    lookupDouble.emplace("Intake.EjectHatch.topSpeed", 0);
    lookupDouble.emplace("Intake.IntakeCargo.bottomSpeed", 0.4);
    lookupDouble.emplace("Intake.IntakeCargo.topSpeed", 1);
    lookupDouble.emplace("Intake.IntakeHatch.bottomSpeed", -0.9);
    lookupDouble.emplace("Intake.IntakeHatch.topSpeed", 0);
    lookupDouble.emplace("Intake.Position.cargopickup", -2000);
    lookupDouble.emplace("Intake.Positition.cargoshot", -800);
    lookupDouble.emplace("Intake.Positition.floor", -2300);
    lookupDouble.emplace("Intake.Positition.levelone", -650);
    lookupDouble.emplace("Intake.Positition.rocketshot", 1100);
    lookupDouble.emplace("Intake.Positition.starting", 1800);
    lookupDouble.emplace("Intake.position.base", 3577);
    lookupDouble.emplace("Intake.position.ffzero", 0.11);
    lookupDouble.emplace("Intake.position.ffzeropos", 6720);
    lookupDouble.emplace("Intake.rotate.A", 400);
    lookupDouble.emplace("Intake.rotate.D", 0);
    lookupDouble.emplace("Intake.rotate.I", 0);
    lookupDouble.emplace("Intake.rotate.P", 1.5);
    lookupDouble.emplace("Intake.rotate.V", 3000);

    lookupBool.emplace("JackScrew.EnableEStop", false);
    lookupDouble.emplace("JackScrew.cr.backSpeed", 1);
    lookupDouble.emplace("JackScrew.cr.frontSpeed", 1);
    lookupDouble.emplace("JackScrew.cr.rampTime", 0.5);
    lookupDouble.emplace("JackScrew.cr.spinUpSpeed", 0.25);
    lookupDouble.emplace("JackScrew.cr.spinUpTime", 0.25);
    lookupDouble.emplace("JackScrew.deltaDown", 0.035);
    lookupDouble.emplace("JackScrew.deltaUp", 0.05);
    lookupDouble.emplace("JackScrew.dist", 98);
    lookupDouble.emplace("JackScrew.dist.L2", 35);
    lookupDouble.emplace("JackScrew.haltDisplacementThreshold", 10);
    lookupDouble.emplace("JackScrew.speedDisplacementThreshold", 1);

    lookupDouble.emplace("JackScrewControl.CloseLoopThreshold", 7);
    lookupDouble.emplace("JackScrewControl.PullUpAmps", 40);
    lookupDouble.emplace("JackScrewControl.PullUpAmpsCounts", 3);
    lookupDouble.emplace("JackScrewControl.PullUpApproachSpeed", -0.2);
    lookupDouble.emplace("JackScrewControl.PullUpApproachThreshold", 12);
    lookupDouble.emplace("JackScrewTest.speed", 0.1);

    lookupDouble.emplace("Lift.step2.drivespeed.y", 0.3);
    lookupDouble.emplace("ProtoScrewSpeed", 10);
    lookupDouble.emplace("PulsesPerInch", 0.4657);
    lookupDouble.emplace("Steer.ContinuousCurrentLimit", 20);
    
    lookupDouble.emplace("TwistD", 0.01);
    lookupDouble.emplace("TwistI", 0);
    lookupDouble.emplace("TwistP", 0.006);

    lookupDouble.emplace("Vision.x.D", 0);
    lookupDouble.emplace("Vision.x.I", 0);
    lookupDouble.emplace("Vision.x.P", 0.015);
    lookupDouble.emplace("Vision.x.range", 0.3);
    lookupDouble.emplace("Vision.x.threshold", 50);

    lookupDouble.emplace("crawl.speed", 1);
}