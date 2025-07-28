#include "datalog_manager.h"
#include <Arduino.h>

DataLogManager::DataLogManager(ILogChannel* logger) : _logger(logger) {}

void DataLogManager::logRaw(const char* line) {
    if (_logger && _logger->isReady()) _logger->log(line);
}

void DataLogManager::logEvent(float missionTime, const char* event, float value1, float value2) {
    char buf[128];
    snprintf(buf, sizeof(buf), "%.3f,%s,%.3f,%.3f", missionTime, event, value1, value2);
    logRaw(buf);
}

void DataLogManager::logSensor(float missionTime, float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ) {
    char buf[128];
    snprintf(buf, sizeof(buf), "%.3f,SENSOR,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
             missionTime, accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
    logRaw(buf);
}

void DataLogManager::flush() {
    if (_logger) _logger->flush();
}

void DataLogManager::logHeader() {
    _logger->log("TIME,EVENT,VAL1,VAL2");
    _logger->log("TIME,EVENT,ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z");
}