#include "MavlinkManager.hpp"
#include <iostream>

long mapValue(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

MavlinkManager::MavlinkManager(SerialPort& port) : serial(port) {}

bool MavlinkManager::sendPacket(mavlink_message_t* msg) {
    if (!serial.isConnected()) return false;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    return serial.writeDataBytes(reinterpret_cast<const char*>(buf), len);
}

bool MavlinkManager::update(Telemetry& tele, PIDConfig& pidConfig) {
    if (!serial.isConnected()) return false;

    uint8_t buffer[512];
    int bytesRead = serial.readData(reinterpret_cast<char*>(buffer), sizeof(buffer));

    if (bytesRead <= 0) return false;

    bool messageReceived = false;

    for (int i = 0; i < bytesRead; i++) {
        mavlink_message_t msg;
        mavlink_status_t status;

        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
            messageReceived = true; // Am primit cu succes un pachet!

            switch (msg.msgid) {
                // ---- AICI ERA PROBLEMA (Trebuie să ne asigurăm că citim Heartbeat-ul!) ----
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&msg, &hb);
                    // Forțăm evaluarea exactă a bitului 128 (MAV_MODE_FLAG_SAFETY_ARMED)
                    tele.isArmed = ((hb.base_mode & 128) != 0);
                    break;
                }
                case MAVLINK_MSG_ID_ATTITUDE: {
                    mavlink_attitude_t att;
                    mavlink_msg_attitude_decode(&msg, &att);
                    tele.roll  = att.roll * (180.0f / 3.14159265f);
                    tele.pitch = att.pitch * (180.0f / 3.14159265f);
                    break;
                }
                case MAVLINK_MSG_ID_VFR_HUD: {
                    mavlink_vfr_hud_t hud;
                    mavlink_msg_vfr_hud_decode(&msg, &hud);
                    tele.altitude = hud.alt;
                    tele.throttle = hud.throttle;
                    break;
                }
                case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
                    mavlink_servo_output_raw_t servos;
                    mavlink_msg_servo_output_raw_decode(&msg, &servos);
                    tele.m1 = servos.servo1_raw;
                    tele.m2 = servos.servo2_raw;
                    tele.m3 = servos.servo3_raw;
                    tele.m4 = servos.servo4_raw;
                    break;
                }
                case MAVLINK_MSG_ID_STATUSTEXT: {
                    mavlink_statustext_t txt;
                    mavlink_msg_statustext_decode(&msg, &txt);
                    std::cout << "\n[DRONA] " << txt.text << std::endl;
                    break;
                }
                case MAVLINK_MSG_ID_RADIO_STATUS: {
                    mavlink_radio_status_t radioStat;
                    mavlink_msg_radio_status_decode(&msg, &radioStat);
                    tele.rssi = radioStat.rssi; // Salvăm procentajul
                    std::cout << "[GCS] Semnal LoRa Hardware: -" << radioStat.rxerrors
                              << " dBm | Calitate: " << (int)radioStat.rssi << "%" << std::endl;
                    break;
                }
                case MAVLINK_MSG_ID_PARAM_VALUE: {
                    mavlink_param_value_t param;
                    mavlink_msg_param_value_decode(&msg, &param);
                    
                    std::string param_id(param.param_id, strnlen(param.param_id, 16));
                    
                    if (param_id == "RATE_ROLL_P") pidConfig.rollP = param.param_value;
                    else if (param_id == "RATE_ROLL_I") pidConfig.rollI = param.param_value;
                    else if (param_id == "RATE_ROLL_D") pidConfig.rollD = param.param_value;
                    else if (param_id == "RATE_PITCH_P") pidConfig.pitchP = param.param_value;
                    else if (param_id == "RATE_PITCH_I") pidConfig.pitchI = param.param_value;
                    else if (param_id == "RATE_PITCH_D") pidConfig.pitchD = param.param_value;
                    else if (param_id == "RATE_YAW_P") pidConfig.yawP = param.param_value;
                    else if (param_id == "RATE_YAW_I") pidConfig.yawI = param.param_value;
                    else if (param_id == "RATE_YAW_D") pidConfig.yawD = param.param_value;
                    
                    std::cout << "[GCS] PID Param Received: " << param_id << " = " << param.param_value << std::endl;
                    break;
                }
            }
        }
    }
    return messageReceived;
}

void MavlinkManager::sendManualControl(int pitch, int roll, int throttle, int yaw, uint16_t buttons) {
    mavlink_message_t msg;
    mavlink_msg_manual_control_pack(
        gcsSysId, gcsCompId, &msg, targetSysId,
        static_cast<int16_t>(pitch),
        static_cast<int16_t>(roll),
        static_cast<int16_t>(throttle),
        static_cast<int16_t>(yaw),
        buttons,
        0,0,0,0,0,0,0,0,0,0 // extensii suplimentare pentru butoane 2, etc. (lăsăm 0)
    );
    sendPacket(&msg);
}

void MavlinkManager::sendHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(gcsSysId, gcsCompId, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    sendPacket(&msg);
}

void MavlinkManager::requestTelemetry() {
    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(gcsSysId, gcsCompId, &msg, targetSysId, targetCompId, MAV_DATA_STREAM_ALL, 10, 1);
    sendPacket(&msg);
}

void MavlinkManager::sendParamSet(const char* param_id, float param_value) {
    mavlink_message_t msg;
    char param_buf[16] = {0};
    strncpy(param_buf, param_id, 16);
    
    mavlink_msg_param_set_pack(gcsSysId, gcsCompId, &msg, targetSysId, targetCompId,
                               param_buf, param_value, MAV_PARAM_TYPE_REAL32);
    sendPacket(&msg);
    std::cout << "[GCS] Sent PARAM_SET: " << param_buf << " = " << param_value << std::endl;
}

void MavlinkManager::sendParamRequestList() {
    mavlink_message_t msg;
    mavlink_msg_param_request_list_pack(gcsSysId, gcsCompId, &msg, targetSysId, targetCompId);
    sendPacket(&msg);
    std::cout << "[GCS] Sent PARAM_REQUEST_LIST to Drone." << std::endl;
}