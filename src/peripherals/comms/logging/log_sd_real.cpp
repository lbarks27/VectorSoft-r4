#include "log_sd_real.h"
#include <Arduino.h>

LogSDReal::LogSDReal(int csPin, const char* logDir, const char* logPrefix)
    : _csPin(csPin), _logDir(logDir), _logPrefix(logPrefix), _ok(false), _lastError(LogSDError::NONE)
{
    _filename[0] = '\0';
}

bool LogSDReal::begin() {
    _ok = SD.begin(_csPin);
    if (!_ok) {
        setError(LogSDError::SD_INIT_FAIL);
        Serial.println("[SD] ERROR: SD card init failed!");
        return false;
    }
    if (!SD.exists(_logDir)) {
        if (!SD.mkdir(_logDir)) {
            setError(LogSDError::DIR_FAIL);
            Serial.println("[SD] ERROR: Log directory creation failed!");
            return false;
        }
    }
    selectNextLogFile();
    _file = SD.open(_filename, FILE_WRITE);
    if (!_file) {
        setError(LogSDError::FILE_OPEN_FAIL);
        Serial.print("[SD] ERROR: Cannot open log file: "); Serial.println(_filename);
        _ok = false;
        return false;
    }
    _ok = true;
    setError(LogSDError::NONE);
    Serial.print("[SD] Log file: "); Serial.println(_filename);
    return true;
}

void LogSDReal::log(const char* msg) {
    if (!_ok || !_file) {
        setError(LogSDError::FILE_WRITE_FAIL);
        Serial.println("[SD] ERROR: Write attempted without open log file.");
        return;
    }
    if (_file.println(msg) == 0) {
        setError(LogSDError::FILE_WRITE_FAIL);
        Serial.println("[SD] ERROR: File write failed.");
    }
}

void LogSDReal::flush() {
    if (_ok && _file) {
        _file.flush();
    }
}

bool LogSDReal::isReady() const {
    // Cast away const for File's non-const operator bool()
    return _ok && const_cast<File&>(_file);
}

const char* LogSDReal::currentLogFilename() const { return _filename; }

// --- Read API ---
bool LogSDReal::openRead(const char* filename) {
    if (_readFile) _readFile.close();
    _readFile = SD.open(filename, FILE_READ);
    if (!_readFile) {
        setError(LogSDError::FILE_OPEN_FAIL);
        Serial.print("[SD] ERROR: Cannot open for read: "); Serial.println(filename);
        return false;
    }
    return true;
}

int LogSDReal::readLine(char* buffer, size_t maxlen) {
    if (!_readFile) return -1;
    size_t n = 0;
    while (_readFile.available() && n < maxlen - 1) {
        char c = _readFile.read();
        if (c == '\n') break;
        buffer[n++] = c;
    }
    buffer[n] = 0;
    if (n == 0 && !_readFile.available()) return -1; // EOF
    return n;
}

void LogSDReal::closeRead() {
    if (_readFile) _readFile.close();
}

// --- Error/Status ---
void LogSDReal::setError(LogSDError err) { _lastError = err; }
LogSDError LogSDReal::lastError() const { return _lastError; }
const char* LogSDReal::errorString() const {
    switch (_lastError) {
        case LogSDError::NONE:           return "None";
        case LogSDError::SD_INIT_FAIL:   return "SD init failed";
        case LogSDError::DIR_FAIL:       return "Log directory creation failed";
        case LogSDError::FILE_OPEN_FAIL: return "File open failed";
        case LogSDError::FILE_WRITE_FAIL:return "File write failed";
        case LogSDError::FILE_FLUSH_FAIL:return "File flush failed";
        default:                         return "Unknown";
    }
}

// --- File listing/deleting ---
void LogSDReal::listLogFiles() const {
    File dir = SD.open(_logDir);
    if (!dir) {
        Serial.print("[SD] ERROR: Cannot open directory ");
        Serial.println(_logDir);
        return;
    }
    Serial.print("[SD] Log files in ");
    Serial.println(_logDir);

    File entry;
    while ((entry = dir.openNextFile())) {
        if (!entry.isDirectory()) {
            Serial.print("  ");
            Serial.print(entry.name());
            Serial.print(" (");
            Serial.print(entry.size());
            Serial.println(" bytes)");
        }
        entry.close();
    }
    dir.close();
}

bool LogSDReal::deleteLogFile(const char* filename) {
    if (SD.exists(filename)) {
        if (SD.remove(filename)) {
            Serial.print("[SD] Deleted file: ");
            Serial.println(filename);
            return true;
        } else {
            Serial.print("[SD] ERROR: Failed to delete file: ");
            Serial.println(filename);
        }
    } else {
        Serial.print("[SD] File not found: ");
        Serial.println(filename);
    }
    return false;
}

// --- Log file auto-increment logic ---
void LogSDReal::selectNextLogFile() {
    int idx = 1;
    for (; idx < 1000; ++idx) {
        snprintf(_filename, sizeof(_filename), "%s/%s%03d.csv", _logDir, _logPrefix, idx);
        if (!SD.exists(_filename)) break;
    }
    if (idx >= 1000) idx = 999;
    snprintf(_filename, sizeof(_filename), "%s/%s%03d.csv", _logDir, _logPrefix, idx);
}