#include "gnc/control/manager/control_manager.h"
#include <cstring>  // for memcpy

ControlManager::ControlManager(HardwareSerial& comms)
: comms_(comms)
{}

void ControlManager::init() {
    comms_.begin(57600);
    shared_.state = ControlState::Initializing;

    // Start in Standby
    currentMode_ = &standbyMode_;
    currentMode_->onEnter(shared_.current_cmd);
}

void ControlManager::update(uint32_t timestamp_ms) {
    processIncoming();

    if (shared_.state == ControlState::Running && currentMode_) {
        currentMode_->update(shared_.current_cmd,
                             shared_.telemetry,
                             timestamp_ms);
    }

    publishTelemetry();
}

void ControlManager::handleMessage(const uint8_t* data, size_t len) {
    if (len == sizeof(ControlCommand)) {
        ControlCommand cmd;
        memcpy(&cmd, data, len);
        shared_.current_cmd = cmd;

        // decide desired mode
        auto desired = (cmd.throttle > 0.01f)
                     ? ControlTelemetry::Mode::ACTIVE
                     : ControlTelemetry::Mode::STANDBY;

        if (!currentMode_ || currentMode_->getModeId() != desired) {
            currentMode_->onExit();
            currentMode_ = (desired == ControlTelemetry::Mode::ACTIVE)
                         ? static_cast<IControlMode*>(&activeMode_)
                         : static_cast<IControlMode*>(&standbyMode_);
            currentMode_->onEnter(cmd);
        }

        shared_.state = ControlState::Running;
    }
    else {
        shared_.state = ControlState::Error;
        shared_.telemetry.fault_flag = true;
    }
}

const ControlTelemetry& ControlManager::getTelemetry() const {
    return shared_.telemetry;
}

void ControlManager::processIncoming() {
    if (comms_.available() >= static_cast<int>(sizeof(ControlCommand))) {
        uint8_t buf[sizeof(ControlCommand)];
        comms_.readBytes(buf, sizeof(buf));
        handleMessage(buf, sizeof(buf));
    }
}

void ControlManager::publishTelemetry() {
    comms_.write(reinterpret_cast<const uint8_t*>(&shared_.telemetry),
                  sizeof(shared_.telemetry));
}