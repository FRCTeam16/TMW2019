#include "Lift/JackScrewTest.h"
#include "Robot.h"
#include "Util/PrefUtil.h"
#include <iostream>

JackScrewTest::JackScrewTest() {
    PrefUtil::getSet("JackScrewTest.speed", 0.1);
}


void JackScrewTest::Run() {
    const double speed = PrefUtil::getSet("JackScrewTest.speed", 0.1);
    std::cout << "JackScrewTest::Run(" << speed << ")\n";
    auto wheels = Robot::driveBase->GetWheels();
    wheels.FL->UseOpenLoopDrive(speed);
    wheels.FR->UseOpenLoopDrive(speed);
    wheels.RL->UseOpenLoopDrive(speed);
    wheels.RR->UseOpenLoopDrive(speed);
}