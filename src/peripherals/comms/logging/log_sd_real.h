#pragma once
#include "log_interface.h"
#include <SD.h>

enum class LogSDError {
    NONE,
    SD_INIT_FAIL,
    DIR_FAIL,
    FILE_OPEN_FAIL,
    FILE_WRITE_FAIL,
    FILE_FLUSH_FAIL,
    UNKNOWN
};

class LogSDReal : public ILogChannel {
public:
    LogSDReal(int csPin, const char* logDir = "/logs", const char* logPrefix = "log_");
    bool begin() override;
    void log(const char* msg) override;
    void flush() override;
    bool isReady() const override;  // Must be const!

    // File reading
    bool openRead(const char* filename) override;
    int readLine(char* buffer, size_t maxlen) override;
    void closeRead() override;

    // File management
    void listLogFiles() const;
    bool deleteLogFile(const char* filename);

    // Status
    LogSDError lastError() const;
    const char* errorString() const;
    const char* currentLogFilename() const;

private:
    int _csPin;
    const char* _logDir;
    const char* _logPrefix;
    char _filename[32];
    File _file;
    File _readFile;
    bool _ok;
    LogSDError _lastError;

    void selectNextLogFile();
    void setError(LogSDError err);
};