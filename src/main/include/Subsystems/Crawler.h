#pragma once

#include "SubsystemManager.h"
#include <frc/Timer.h>
#include "Util/RampUtil.h"
#include "Util/BSPrefs.h"
#include <iostream>

class Crawler : public SubsystemManager {
private:
    bool enabled = true;
    double startTime = -1.0;        // time drive move starts (for ramping)
    double crawlSpeed = -1.0;       // default speed
    double targetSpeed = 0.0;       // speed target (fwd/rev) 
    const double kRampTime = 0.2;   // time to ramp up

    void Crawl(double _speed) {
        if (startTime == -1.0) {
            startTime = frc::Timer::GetFPGATimestamp();
        }
        targetSpeed = _speed;
    }

public:
    Crawler() = default;

    void Init() override {
        crawlSpeed = BSPrefs::GetInstance()->GetDouble("crawl.speed", 1.0);
    }

    void Run() override {
        double speed = 0.0;
        if (startTime > -1.0) {
            const double now = frc::Timer::GetFPGATimestamp();
            const int direction = targetSpeed < 0 ? -1 : 1;
            speed = RampUtil::RampUp(fabs(targetSpeed), (now - startTime), kRampTime, 0.0) * direction;
        }
        if (!enabled) {
            speed = 0.0;
            startTime = -1;
        }
        RobotMap::crawlMotor->Set(speed);
    }

    void Forward() {
        Crawl(-crawlSpeed);
    }

    void Back() {
        Crawl(crawlSpeed);
    }

    void Stop() {
        startTime = -1;
        targetSpeed = 0.0;
    }

    void SetCrawlEnabled(bool _enabled) {
        std::cout << "Crawler::SetCrawlEnabled(" << _enabled << ")\n";
        enabled = _enabled;
    }
};