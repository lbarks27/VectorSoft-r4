#pragma once

#include "control_interface.h"
#include <HardwareSerial.h>

/**
 * @file control_manager.h
 * @brief Declares ControlManager (no base interface).
 */

class ControlManager {
public:
    ControlManager(HardwareSerial& comms = Serial2);

    /** One-time hardware/service setup */
    void init();

    /** Called every main loop iteration */
    void update(uint32_t timestamp_ms);

    /** Feed in raw command packets */
    void handleMessage(const uint8_t* data, size_t len);

    /** Expose telemetry for logging/groundstation */
    const ocntrolTelemetry& getTelemetry() const;

private:
    HardwareSerial& comms_;
    ControlShared   shared_;

    // Modes
    StandbyMode     standbyMode_;
    ActiveMode      activeMode_;
    IControlMode*   currentMode_ = nullptr;

    // Internal steps
    void processIncoming();
    void publishTelemetry();
};