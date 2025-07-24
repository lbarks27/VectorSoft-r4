#pragma once
#include "log_interface.h"

class DataLogManager {
public:
    DataLogManager(ILogChannel* logger);

    void logRaw(const char* line);

    void logEvent(float missionTime, const char* event, float value1 = 0.0f, float value2 = 0.0f);

    void logSensor(float missionTime, float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ);

    void flush();

    void logHeader();

private:
    ILogChannel* _logger;
};