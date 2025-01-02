/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 10:40:08
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2025-01-02 16:11:11
 * @FilePath: /UAVtoController/src/Application.cpp
 * @Description: 
 */
#include "Application.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

Application* Application::_instance = nullptr;

Application::Application(int argc, char **argv) : ProtocolObserver()
, _mavlink(nullptr)
, _serialLink(nullptr)
, _ctrlCenter(nullptr)
, _udpLink(nullptr)
{
    _instance = this;
}

Application::~Application()
{
    _quit = true;
    _instance = nullptr;
    if (_serialLink) {
        delete _serialLink;
        _serialLink = nullptr;
    }
    if (_mavlink) {
        delete _mavlink;
        _mavlink = nullptr;
    }

    if (_ctrlCenter) {
        delete _ctrlCenter;
        _ctrlCenter = nullptr;
    }
    if (_udpLink) {
        delete _udpLink;
        _udpLink = nullptr;
    }
}

bool Application::init()
{
    _mavlink = new ProtocolMavlink(this);
    _serialLink = new SerialLink(_mavlink);

    bool ok = _serialLink->open(_argSerialPort, _argBaudRate);
    if (!ok) return false;

    _ctrlCenter = new ProtocolCtrlCenter(this);
    _udpLink = new UDPLink(_ctrlCenter);
    ok = _udpLink->connectLink();
    if (!ok) return false;
    return true;
}

void Application::quit()
{
    _quit = true;
    if (_serialLink && _serialLink->isConnected()) {
        _serialLink->disconnectLink();
    }
    if (_udpLink && _udpLink->isConnected()) {
        _udpLink->disconnectLink();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void Application::exec()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto last_heartbeat = now;
    _sendHeartbeat();
    while (!_quit) {
        now = std::chrono::high_resolution_clock::now();
        if (_serialLink && !_serialLink->isConnected()) {
            bool opened = _serialLink->open(_argSerialPort, _argBaudRate);
        }

        if (std::chrono::duration<float, std::milli>(now - last_heartbeat).count() > 1000) {
            last_heartbeat = now;
            _sendHeartbeat();
        }
    }
}

void Application::onMavlinkMessageReceive(const mavlink_message_t &message)
{
    if (message.sysid == 255) return;
    std::cout << "msgid:" << message.msgid << std::endl;
    switch (message.msgid)
    {
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        _handleHeartbeat(message);
    }
    break;
    case MAVLINK_MSG_ID_ATTITUDE:
    {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&message, &attitude);
        _pitch = attitude.pitch * (180/M_PI);
        _roll = attitude.roll * (180/M_PI);
        _yaw = attitude.yaw * (180/M_PI);
        if (_yaw < 0.0) {
            _yaw += 360.0;
        }

        _angleSpeed_x = attitude.rollspeed * (180/M_PI);
        _angleSpeed_y = attitude.pitchspeed * (180/M_PI);
        _angleSpeed_z = attitude.yawspeed * (180/M_PI);
    }
    break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
        mavlink_global_position_int_t position;
        mavlink_msg_global_position_int_decode(&message, &position);
        _altitude = position.alt / 1000.0;
        _relativeAltitude = position.relative_alt / 1000.0;
        _longitude = position.lon / (double)1E7;
        _latitude = position.lat / (double)1E7;

        // 发送数据到指控
        _sendPositionTOCtrlCenter();
    }
    break;
    case MAVLINK_MSG_ID_SYS_STATUS:
    {
        _handleSysStatus(message);
    }
    break;
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
    {
        _handleLocalPositionNED(message);
    }
    break;
    case MAVLINK_MSG_ID_RAW_IMU:
    {
        _handleRawIMU(message);
    }
    break;
    case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
    {
        _handlePositionTargetLocalNED(message);
    }
    break;
    default:
        break;
    }
}

void Application::onCtrlCenterMessageReceive(const ctrl_center_message_t &message)
{
    std::cout << __func__ << " " << __LINE__ << std::endl;
    switch (message.type)
    {
    case CTRL_CENTER_MSG_TYPE_COMMAND:
    {
        ctrl_center_command_long_t command_long;
        ctrl_center_msg_command_long_decode(&message, &command_long);
        // TODO : 收到指控的数据后进行相应的处理
        if (command_long.mode == 0) {// 0-待机 
            // TODO
        } else if (command_long.mode == 1) {// 1-作战 
            // TODO
        } else if (command_long.mode == 2) {// 2-悬停
            _setFlightMode("Loiter");
        } else if (command_long.mode == 3) {// 3-一键起飞 
            // TODO
        } else if (command_long.mode == 4) {// 4-一键返航 
            _setFlightMode("RTL");
        } else if (command_long.mode == 5) {// 5-原地降落 
            _setFlightMode("Land");
        } else if (command_long.mode == 6) {// 6-自主巡航 
            // TODO
        } else if (command_long.mode == 7) {// 7-定高，需要发送高度信息
            _setFlightMode("ALT_HOLD");
        } else if (command_long.mode == 8) {// 8-遥操作模式
            // TODO
        }
    }
    break;
    
    default:
        break;
    }
}

void Application::_sendHeartbeat()
{
    if (_serialLink && _serialLink->isConnected() && _receiveHeartbeat) {
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {0};
        mavlink_msg_heartbeat_pack(255, MAV_COMP_ID_MISSIONPLANNER, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
        int len = mavlink_msg_to_send_buffer(buf, &msg);
        _serialLink->writeData(buf, len);
    }
}

void Application::_setFlightMode(const std::string &mode)
{
    uint8_t     base_mode;
    uint32_t    custom_mode;

    if (_flightModeDecode.setFlightMode(mode, base_mode, custom_mode)) {
        uint8_t newBaseMode = _base_mode & ~MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE;

        newBaseMode |= base_mode;

        if (_serialLink && _serialLink->isConnected()) {
            mavlink_message_t msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {0};
            mavlink_msg_set_mode_pack(255, MAV_COMP_ID_MISSIONPLANNER, &msg, _targetSystemId, newBaseMode, custom_mode);
            int len = mavlink_msg_to_send_buffer(buf, &msg);
            _serialLink->writeData(buf, len);
        }
    }
}

void Application::_requestDataStream(MAV_DATA_STREAM stream, uint16_t rate)
{
    if (_serialLink && _serialLink->isConnected()) {
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {0};
        mavlink_msg_request_data_stream_pack(255, MAV_COMP_ID_MISSIONPLANNER, &msg, _targetSystemId, _targetComponentId, stream, rate, 1);
        int len = mavlink_msg_to_send_buffer(buf, &msg);
        _serialLink->writeData(buf, len);
    }    
}

void Application::_handleHeartbeat(const mavlink_message_t &msg)
{
    _receiveHeartbeat = true;

    static bool requestDataStream = false;
    if (_targetSystemId != msg.sysid || _targetComponentId != msg.compid) {
        _targetSystemId = msg.sysid;
        _targetComponentId = msg.compid;
        requestDataStream = false;
    }
    if (!requestDataStream) {
        requestDataStream = true;
        _requestDataStream(MAV_DATA_STREAM_RAW_SENSORS,     2);
        _requestDataStream(MAV_DATA_STREAM_EXTENDED_STATUS, 2); // msgid: 125,152,62, 42,24
        _requestDataStream(MAV_DATA_STREAM_RC_CHANNELS,     2);
        _requestDataStream(MAV_DATA_STREAM_POSITION,        3); // msgid: 32,33   位置
        _requestDataStream(MAV_DATA_STREAM_EXTRA1,          4);   // msgid: 30,164,178   姿态
        _requestDataStream(MAV_DATA_STREAM_EXTRA2,          4);
        _requestDataStream(MAV_DATA_STREAM_EXTRA3,          3); // msgid:2,163,165,136,193, 241,147,22
    }

    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&msg, &heartbeat);

    bool newArmed = heartbeat.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY;

    if (_armed != newArmed) {
        _armed = newArmed;
    }

    if (_firmwareType != heartbeat.autopilot || _vehicleType != heartbeat.type) {
        _firmwareType = heartbeat.autopilot;
        _vehicleType = heartbeat.type;
        _flightModeDecode.init(_firmwareType, _vehicleType);
    }

    if (heartbeat.base_mode != _base_mode || heartbeat.custom_mode != _custom_mode) {
        _base_mode   = heartbeat.base_mode;
        _custom_mode = heartbeat.custom_mode;
        _flightMode = _flightModeDecode.flightMode(_base_mode, _custom_mode);
        std::cout << " flight mode changed: " << _flightMode << std::endl;
    }
}

void Application::_handleSysStatus(const mavlink_message_t &msg)
{
    mavlink_sys_status_t sysStatus;
    mavlink_msg_sys_status_decode(&msg, &sysStatus);
    float voltage_battery;//：电压
    if (sysStatus.current_battery == -1) {
        _currentBattery = 0;
    } else {
        _currentBattery = sysStatus.current_battery / 100.0f;
    }
    if (sysStatus.voltage_battery == UINT16_MAX) {
        _voltageBattery = 0;
    } else {
        _voltageBattery = sysStatus.voltage_battery / 1000.0f;
    }
    _batteryRemaining = sysStatus.battery_remaining;//剩余电量

    bool isHealth = (sysStatus.onboard_control_sensors_enabled & sysStatus.onboard_control_sensors_health) == sysStatus.onboard_control_sensors_enabled;
    if (isHealth) {
        _allSensorsHealthy = 0;
    } else {
        //_allSensorsHealthy = onboard_control_sensors_health;
        // if (sysStatus.onboard_control_sensors_enabled & MAV_SYS_STATUS_PREARM_CHECK) {
        //     if (!_readyToFlyAvailable) {
        //         _readyToFlyAvailable = true;
        //     }

        //     bool newReadyToFly = sysStatus.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK;
        //     if (newReadyToFly != _readyToFly) {
        //         _readyToFly = newReadyToFly;
        //     }
        // }
        _allSensorsHealthy = 1;
    }
    

    _sendBatteryInfoToCtrlCenter();
    _sendSysStatusTOCtrlCenter();
}

void Application::_handleLocalPositionNED(const mavlink_message_t &msg)
{
    mavlink_local_position_ned_t localPositionNED;
    mavlink_msg_local_position_ned_decode(&msg, &localPositionNED);
    _velocityNED_x = localPositionNED.vx;
    _velocityNED_y = localPositionNED.vy;
    _velocityNED_z = localPositionNED.vz;
}

void Application::_handleRawIMU(const mavlink_message_t &msg)
{
    // mavlink_raw_imu_t rawIMU;
    // mavlink_msg_raw_imu_decode(&msg, &rawIMU);
}

void Application::_handlePositionTargetLocalNED(const mavlink_message_t &msg)
{
    mavlink_position_target_local_ned_t positionTargetLocalNED;
    mavlink_msg_position_target_local_ned_decode(&msg, &positionTargetLocalNED);
    _acceleratedSpeed_x = positionTargetLocalNED.afx;
    _acceleratedSpeed_y = positionTargetLocalNED.afy;
    _acceleratedSpeed_z = positionTargetLocalNED.afz;

    // _velocityNED_x = positionTargetLocalNED.vx;
    // _velocityNED_y = positionTargetLocalNED.vy;
    // _velocityNED_z = positionTargetLocalNED.vz;
}

// TODO : 发送给指控的函数在下面添加

void Application::_sendPositionTOCtrlCenter()
{
    uint8_t buf[CTRL_CENTER_MSG_FRAME_LEN_MAX] = {0};
    ctrl_center_message_t message;
    ctrl_center_msg_position_pack(_longitude, _latitude, _altitude, message);
    int length = ctrl_center_msg_to_send_buffer(buf, message);
    if (_udpLink) {
        _udpLink->writeData(buf, length);
    }
}

void Application::_sendSysStatusTOCtrlCenter()
{
    uint8_t buf[CTRL_CENTER_MSG_FRAME_LEN_MAX] = {0};
    ctrl_center_message_t message;
    ctrl_center_sys_status_t sysStatus;
    sysStatus.rtkFixType = _rtkFixType;
    sysStatus.relativeAltitude = _relativeAltitude * 10;
    sysStatus.velocity.vx = _velocityNED_x * 100;
    sysStatus.velocity.vy = _velocityNED_y * 100;
    sysStatus.velocity.vz = _velocityNED_z * 100;
    sysStatus.accSpeed.accx = _acceleratedSpeed_x * 1000;
    sysStatus.accSpeed.accy = _acceleratedSpeed_y * 1000;
    sysStatus.accSpeed.accz = _acceleratedSpeed_z * 1000;
    sysStatus.roll = _roll * 100;
    sysStatus.pitch = _pitch * 100;
    sysStatus.yaw = _yaw * 100;
    sysStatus.angleVelocity.vx = _angleSpeed_x * 100;
    sysStatus.angleVelocity.vy = _angleSpeed_y * 100;
    sysStatus.angleVelocity.vz = _angleSpeed_z * 100;
    sysStatus.allSensorsHealthy = _allSensorsHealthy;
    sysStatus.commandAck = _commandAck;
    ctrl_center_msg_sys_status_pack(sysStatus, message);
    int length = ctrl_center_msg_to_send_buffer(buf, message);
    if (_udpLink) {
        _udpLink->writeData(buf, length);
    }

    _commandAck = 0;
}

void Application::_sendBatteryInfoToCtrlCenter()
{
    uint8_t buf[CTRL_CENTER_MSG_FRAME_LEN_MAX] = {0};
    ctrl_center_message_t message;
    double tmp = _voltageBattery * _currentBattery; // w
    tmp = tmp * 0.001; // kw
    uint16_t power = tmp * 100;
    ctrl_center_msg_power_pack(power, message);
    int length = ctrl_center_msg_to_send_buffer(buf, message);
    if (_udpLink) {
        _udpLink->writeData(buf, length);
    }
}

Application *Application::getInstance(void)
{
    return Application::_instance;
}