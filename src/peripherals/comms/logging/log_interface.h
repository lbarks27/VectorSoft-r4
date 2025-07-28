#pragma once
#include <cstddef>

class ILogChannel {
public:
    virtual bool begin() = 0;
    virtual void log(const char* msg) = 0;
    virtual void flush() = 0;
    virtual bool isReady() const = 0;

    // Optional: File reading
    virtual bool openRead(const char* filename) = 0;
    virtual int readLine(char* buffer, size_t maxlen) = 0;
    virtual void closeRead() = 0;

    virtual ~ILogChannel() {}
};