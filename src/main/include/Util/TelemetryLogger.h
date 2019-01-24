#ifndef SRC_UTIL_TELEMETRYLOGGER_H_
#define SRC_UTIL_TELEMETRYLOGGER_H_

#include "frc/WPILib.h"
#include <fstream>

class TelemetryLogger {
public:
	TelemetryLogger();
	virtual ~TelemetryLogger();
	void Run();
	void Begin();
	void End();
	void Launch();
private:
	std::thread telemetryThread;
	bool running = false;
	std::ofstream logFile;
	void Log();
};

#endif /* SRC_UTIL_TELEMETRYLOGGER_H_ */
