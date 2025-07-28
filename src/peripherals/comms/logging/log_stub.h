#pragma once
#include "log_interface.h"
#include <Arduino.h>

class LogStub : public ILogChannel {
public:
    bool begin() override { Serial.println("LogStub BEGIN"); return true; }
    void log(const char* msg) override { Serial.print("LogStub: "); Serial.println(msg); }
    void flush() override {}
    bool isReady() const override { return true; }

    // File reading/dummy
    bool openRead(const char*) override { Serial.println("LogStub: openRead"); return true; }
    int readLine(char* buf, size_t maxlen) override {
        Serial.println("LogStub: readLine");
        if (maxlen > 0) buf[0] = 0;
        return -1;
    }
    void closeRead() override { Serial.println("LogStub: closeRead"); }
};