#ifndef MAVLINK_MANAGER_HPP
#define MAVLINK_MANAGER_HPP

#include "SerialPort.hpp"
#include "Telemetry.hpp"
#include "PIDConfig.hpp"
#include "mavlink/common/mavlink.h"

class MavlinkManager {
private:
    SerialPort& serial;
    uint8_t gcsSysId = 255;
    uint8_t gcsCompId = 190;
    uint8_t targetSysId = 1;
    uint8_t targetCompId = 1;

    bool sendPacket(mavlink_message_t* msg);

public:
    MavlinkManager(SerialPort& port);

    bool update(Telemetry& tele, PIDConfig& pidConfig);
    void sendManualControl(int pitch, int roll, int throttle, int yaw, uint16_t buttons);
    void sendHeartbeat();
    void requestTelemetry();
    
    // PID Param methods
    void sendParamSet(const char* param_id, float param_value);
    void sendParamRequestList();
};

#endif