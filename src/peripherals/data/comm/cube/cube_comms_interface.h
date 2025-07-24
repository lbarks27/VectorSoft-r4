#pragma once
#include <stdint.h>
#include <stddef.h>
#include <vector>

// Example MAVLink message wrapper (or make your own struct for your messages)
struct CubeMsg {
    uint8_t msgid;
    std::vector<uint8_t> payload;
    // Add timestamp, source, etc. as needed
};

class ICubeComms {
public:
    virtual bool begin() = 0;
    virtual bool send(const CubeMsg& msg) = 0;
    virtual bool receive(CubeMsg& msg) = 0;   // Returns true if a message was received
    virtual bool isHealthy() const = 0;
    virtual ~ICubeComms() {}
};