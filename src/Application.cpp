/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 10:40:08
 * @LastEditors: vincent_xjw@163.com
 * @LastEditTime: 2025-01-19 23:50:02
 * @FilePath: /UAVtoController/src/Application.cpp
 * @Description: 
 */
#include "Application.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include "INIReader.h"

Application* Application::_instance = nullptr;

Application::Application(int argc, char **argv) : ProtocolObserver()
, _mavlink(nullptr)
, _vehicleLink(nullptr)
, _ctrlCenter(nullptr)
, _ctrlCenterLink(nullptr)
, _flightModeInterface(nullptr)
{
    memset(&_command_long, 0, sizeof(ctrl_center_command_long_t));
    _instance = this;
    _bootTimeMs = getCurrentMs();
}

Application::~Application()
{
    _quit = true;
    _instance = nullptr;
    if (_vehicleLink) {
        delete _vehicleLink;
        _vehicleLink = nullptr;
    }
    if (_mavlink) {
        delete _mavlink;
        _mavlink = nullptr;
    }

    if (_ctrlCenter) {
        delete _ctrlCenter;
        _ctrlCenter = nullptr;
    }
    if (_ctrlCenterLink) {
        delete _ctrlCenterLink;
        _ctrlCenterLink = nullptr;
    }
}

void Application::loadSettings()
{
    // 创建INIReader对象并加载配置文件
    const char* home_dir = std::getenv("HOME");
    std::string fileName = std::string(home_dir) + "/.config/uavToController.ini";
    INIReader reader(fileName);

    // 检查文件是否被正确加载
    if (reader.ParseError() < 0) {
        std::cerr << "无法加载INI文件" << std::endl;
        saveSettings();
        return;
    }

    // 读取数据库相关参数
    // 飞机参数
    _vehicleLinkConfig.setType(reader.GetString("VEHICLE", "Type", "serial"));
    _vehicleLinkConfig.setSerialPort(reader.GetString("VEHICLE", "SerialPort", "/dev/ttyS0"));
    _vehicleLinkConfig.setSerialBaudrate(reader.GetInteger("VEHICLE", "SerialRate", 115200));
    _vehicleLinkConfig.setUdpServerIP(reader.GetString("VEHICLE", "UdpHost", "127.0.0.1"));
    _vehicleLinkConfig.setUdpServerPort(reader.GetInteger("VEHICLE", "UdpPort", 18570));
    _vehicleLinkConfig.setUdpLocalPort(reader.GetInteger("VEHICLE", "UdpLocalPort", 14550));
    _vehicleLinkConfig.setUdpIsBindLocalPort(reader.GetBoolean("VEHICLE", "IsBindLocalPort", true));
    // 指控参数
    _ctrlCenterLinkConfig.setType(reader.GetString("CTRLCENTER", "Type", "udp"));
    _ctrlCenterLinkConfig.setSerialPort(reader.GetString("CTRLCENTER", "SerialPort", "/dev/ttyS0"));
    _ctrlCenterLinkConfig.setSerialBaudrate(reader.GetInteger("CTRLCENTER", "SerialRate", 115200));
    _ctrlCenterLinkConfig.setUdpServerIP(reader.GetString("CTRLCENTER", "UdpHost", "127.0.0.1"));
    _ctrlCenterLinkConfig.setUdpServerPort(reader.GetInteger("CTRLCENTER", "UdpPort", 2001));
    _ctrlCenterLinkConfig.setUdpLocalPort(reader.GetInteger("CTRLCENTER", "UdpLocalPort", 2002));
    _ctrlCenterLinkConfig.setUdpIsBindLocalPort(reader.GetBoolean("CTRLCENTER", "IsBindLocalPort", true));

    int level = reader.GetInteger("LOG","LEVEL", 1);
    Logger::getInstance()->setLogLevel((LogLevel)level);

    Logger::getInstance()->log(LOGLEVEL_INFO, " vehicle link cofig:" + _vehicleLinkConfig.toString());
    Logger::getInstance()->log(LOGLEVEL_INFO, " ctrl center link cofig:" + _ctrlCenterLinkConfig.toString());
    Logger::getInstance()->log(LOGLEVEL_INFO, " LogLevel:" + std::to_string(level));
}

void Application::saveSettings()
{
    const char* home_dir = std::getenv("HOME");
    std::string fileName = std::string(home_dir) + "/.config/uavToController.ini";

    std::ofstream iniFile(fileName);

    if (!iniFile.is_open()) {
        std::cerr << "无法打开INI文件进行写入!" << std::endl;
        return;
    }

    // 写入数据库配置
    iniFile << "[VEHICLE]" << std::endl;
    iniFile << _vehicleLinkConfig.toString() << std::endl;
    iniFile << "[CTRLCENTER]" << std::endl;
    iniFile << _ctrlCenterLinkConfig.toString() << std::endl;

    iniFile << "[LOG]" << std::endl;
    iniFile << "LEVEL = " << Logger::getInstance()->logLevel() << std::endl;

    iniFile.close();
    std::cout << "配置已更新并写回到INI文件!" << std::endl;
}

bool Application::init()
{
    loadSettings();

    _mavlink = new ProtocolMavlink(this);
    if (_vehicleLinkConfig.type().compare("serial") == 0) {
        _vehicleLink = new SerialLink(_mavlink, &_vehicleLinkConfig);
    }
    else if (_vehicleLinkConfig.type().compare("udp") == 0)
    {
        _vehicleLink = new UDPLink(_mavlink, &_vehicleLinkConfig);
    }
    bool ok = _vehicleLink->connectLink();
    if (!ok) return false;

    _ctrlCenter = new ProtocolCtrlCenter(this);
    if (_ctrlCenterLinkConfig.type().compare("serial") == 0)
    {
        _ctrlCenterLink = new SerialLink(_ctrlCenter, &_ctrlCenterLinkConfig);
    }
    else if (_ctrlCenterLinkConfig.type().compare("udp") == 0)
    {
        _ctrlCenterLink = new UDPLink(_ctrlCenter, &_ctrlCenterLinkConfig);
    }
    ok = _ctrlCenterLink->connectLink();
    if (!ok) return false;
    return true;
}

void Application::quit()
{
    saveSettings();

    _quit = true;
    _isWorkThreadRunning = false;
    if (_workThread.joinable()) {
        _workThread.join();
    }
    while(!_isWorkThreadExited) { std::this_thread::sleep_for(std::chrono::milliseconds(200)); }

    if (_vehicleLink && _vehicleLink->isConnected()) {
        _vehicleLink->disconnectLink();
    }
    if (_ctrlCenterLink && _ctrlCenterLink->isConnected()) {
        _ctrlCenterLink->disconnectLink();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void Application::exec()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto last_heartbeat = now;

    _isWorkThreadRunning = true;
    _workThread = std::thread(&Application::_workThreadFunction, this, this);
    _workThread.detach();

    _sendHeartbeat();
    while (!_quit) {
        now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<float, std::milli>(now - last_heartbeat).count() > 1000)
        {
            last_heartbeat = now;
            _sendHeartbeat();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

FlightModeInterface *Application::createFlightModeInterface()
{
    if (isMultiRotor()) {
        if (isArduPilotFirmwareClass()) {
            return new ApmMultiRotorFlightMode();
        } else if (isPX4FirmwareClass()) {
            return new PX4MultiRotorFlightMode();
        }
    }
    return nullptr;
}

void Application::onMavlinkMessageReceive(const mavlink_message_t &message)
{
    if (message.sysid == 255) return;
    Logger::getInstance()->log(LOGLEVEL_DEBUG, "msgid:" + std::to_string(message.msgid));
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
    case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
    {
        _handleHilStateQuateRnion(message);
    }
    break;
    case MAVLINK_MSG_ID_HOME_POSITION:
    {
        _handleHomePosition(message);
    }
    break;
    case MAVLINK_MSG_ID_STATUSTEXT:
    {
        _handleStatustext(message);
    }
    break;
    case MAVLINK_MSG_ID_PING:
    {
        _handlePing(message);
    }
    break;
    default:
        break;
    }
}

void Application::setArmed(bool armed)
{
    if (armed != _armed) {
        Logger::getInstance()->log(LOGLEVEL_INFO, "setting " + std::string(armed?"armed":"disArmed"));
        _sendMavCommand(MAV_CMD_COMPONENT_ARM_DISARM, armed ? 1.0f : 0.0f);
    }
}

bool Application::isHeartbeatLost()
{
    if (getCurrentMs() - _lastReceiveHeartbeatTimeMS > 5000) return true;
    return false;
}

void Application::onCtrlCenterMessageReceive(const ctrl_center_message_t &message)
{
    switch (message.type)
    {
    case CTRL_CENTER_MSG_TYPE_COMMAND:
    {
        ctrl_center_command_long_t command_long;
        ctrl_center_msg_command_long_decode(&message, &command_long);
        
        // TODO : 收到指控的数据后进行相应的处理
        if (command_long.mode == 0) {// 0-待机 
            // 上锁
            if (_relativeAltitude < 0.2) {
                setArmed(false);
            }
        }
        else if (command_long.mode == 1)
        { // 1-作战
            // 解锁
            setArmed(true);
        }
        else if (command_long.mode == 2)
        { // 2-悬停
            if (_flightModeInterface) {
                _setFlightMode(_flightModeInterface->loiterMode());
            }
        }
        else if (command_long.mode == 3)
        { // 3-一键起飞
            _guidedTakeoff(10);
        }
        else if (command_long.mode == 4)
        { // 4-一键返航
            if (_flightModeInterface) {
                _setFlightMode(_flightModeInterface->rtlMode());
            }
        }
        else if (command_long.mode == 5)
        { // 5-原地降落
            if (_flightModeInterface) {
                _setFlightMode(_flightModeInterface->landMode());
            }
        }
        else if (command_long.mode == 6)
        { // 6-自主巡航
            // _workThreadFunction(); update position
            // setArmed(true);
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
            _setOffboardMode(2);
        }
        else if (command_long.mode == 7)
        { // 7-定高，需要发送高度信息
            // if (_flightModeInterface) {
            //     _setFlightMode(_flightModeInterface->hoverMode());
            // }
            // 起飞到指定高度
            _guidedTakeoff(command_long.position_1.altitude - _homeAltitude);
        }
        else if (command_long.mode == 8)
        { // 8-遥操作模式
            if (_flightModeInterface) {
                _setFlightMode(_flightModeInterface->stabilizedMode());
            }
        }

        _command_long = command_long;
    }
    break;
    
    default:
        break;
    }
}

uint64_t Application::getCurrentMs()
{
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto value = now_ms.time_since_epoch().count();
    return value;
}

uint64_t Application::getCurrentMicroSec()
{
    auto now = std::chrono::system_clock::now();
    auto now_us = std::chrono::time_point_cast<std::chrono::microseconds>(now);
    auto value = now_us.time_since_epoch().count();
    return value;
}

void Application::_sendHeartbeat()
{
    if (_vehicleLink && _vehicleLink->isConnected()) {
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {0};
        // 地面站 常用
        // mavlink_msg_heartbeat_pack(_systemId, _componentId, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);
        // 板上系统 常用mavros 
        mavlink_msg_heartbeat_pack(_systemId, _componentId, &msg, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);
        int len = mavlink_msg_to_send_buffer(buf, &msg);
        _vehicleLink->writeData(buf, len);
    }
}

void Application::_setFlightMode(const std::string &mode)
{
    uint8_t     base_mode;
    uint32_t    custom_mode;

    if (_flightModeInterface && _flightModeInterface->setFlightMode(mode, base_mode, custom_mode)) {
        uint8_t newBaseMode = _base_mode & ~MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE;

        newBaseMode |= base_mode;

        if (_vehicleLink && _vehicleLink->isConnected()) {
            Logger::getInstance()->log(LOGLEVEL_INFO, "setting flight mode:" + mode);
            mavlink_message_t msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {0};
            mavlink_msg_set_mode_pack(_systemId, _componentId, &msg, _targetSystemId, newBaseMode, custom_mode);
            int len = mavlink_msg_to_send_buffer(buf, &msg);
            _vehicleLink->writeData(buf, len);
        }
    }
}

void Application::_requestDataStream(MAV_DATA_STREAM stream, uint16_t rate)
{
    if (_vehicleLink && _vehicleLink->isConnected()) {
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {0};
        mavlink_msg_request_data_stream_pack(_systemId, _componentId, &msg, _targetSystemId, _targetComponentId, stream, rate, 1);
        int len = mavlink_msg_to_send_buffer(buf, &msg);
        _vehicleLink->writeData(buf, len);
    }    
}

void Application::_sendMavCommand(MAV_CMD command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    if (_vehicleLink && _vehicleLink->isConnected()) {
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {0};
        mavlink_msg_command_long_pack(_systemId, _componentId, &msg, _targetSystemId, _targetComponentId, command, 0,
                                      param1, param2, param3, param4, param5, param6, param7);
        int len = mavlink_msg_to_send_buffer(buf, &msg);
        _vehicleLink->writeData(buf, len);
    } 
}

void Application::_guidedTakeoff(float relAltitude)
{
    if (!_flightModeInterface) return;

    std::thread t([](Application *app, float alt) {
        if (app->isMultiRotor()) {
            if (app->isArduPilotFirmwareClass()) {
                std::string guidedMode = app->_flightModeInterface->guidedMode();
                while(app->_flightMode.compare(guidedMode) != 0) {
                    app->_setFlightMode(guidedMode);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                // int tryCount = 3;
                // while (!app->armed() && tryCount--)
                // {
                //     app->setArmed(true);
                //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                // }
                app->_sendMavCommand(MAV_CMD_NAV_TAKEOFF,
                                     -1,   // No pitch requested
                                     0, 0, // param 2-4 unused
                                     std::numeric_limits<float>::quiet_NaN(),
                                     std::numeric_limits<float>::quiet_NaN(),
                                     std::numeric_limits<float>::quiet_NaN(), // No yaw, lat, lon
                                     alt);
            } else if (app->isPX4FirmwareClass()) {
                app->_sendMavCommand(MAV_CMD_NAV_TAKEOFF,
                                     -1,   // No pitch requested
                                     0, 0, // param 2-4 unused
                                     std::numeric_limits<float>::quiet_NaN(),
                                     std::numeric_limits<float>::quiet_NaN(),
                                     std::numeric_limits<float>::quiet_NaN(), // No yaw, lat, lon
                                     alt + app->_homeAltitude);
                // int tryCount = 3;
                // while (!app->armed() && tryCount--)
                // {
                //     app->setArmed(true);
                //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                // }
            }

            // if (!app->armed()) {
            //     Logger::getInstance()->log(LOGLEVEL_INFO, "setArmed failed. And try more then 3.");
            // }
        }
    }, this, relAltitude); // 创建线程，传递 Lambda 表达式和参数
    t.join(); // 等待线程完成
}

void Application::_requestHomePosition()
{
    _sendMavCommand(MAV_CMD_GET_HOME_POSITION);
}

void Application::_setOffboardMode(float height)
{
    if (!_flightModeInterface) {
        return;
    }
        
    std::thread t([](Application *app, float alt)
                  {
        if (app->isMultiRotor()) {
            if (app->isPX4FirmwareClass()) {
                std::string guidedMode = app->_flightModeInterface->guidedMode();
                for (int i = 100; i > 0; --i)
                {
                    // set_position_target_local_ned
                    if (app->_vehicleLink && app->_vehicleLink->isConnected())
                    {
                        mavlink_message_t msg;
                        uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {0};
                        uint32_t bootTimeMs = app->getCurrentMs() - app->_bootTimeMs;
                        // TODO TEST
                        // mavlink_msg_set_position_target_local_ned_pack(app->_systemId, app->_componentId, &msg, bootTimeMs,
                        //                                                app->_targetSystemId, app->_targetComponentId, MAV_FRAME_LOCAL_NED, 0, 0, 0,
                        //                                                alt, 0, 0, 1, 0, 0, 1, 0, 0);
                        mavlink_msg_set_position_target_global_int_pack(app->_systemId, app->_componentId, &msg, bootTimeMs,
                                                                        app->_targetSystemId, app->_targetComponentId, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0,
                                                                        app->_latitude * 1e7, app->_longitude * 1e7, alt, 0, 0, 0, 0, 0, 0, 0, 0);
                        int len = mavlink_msg_to_send_buffer(buf, &msg);
                        app->_vehicleLink->writeData(buf, len);
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                int tryCount = 3;
                while (app->_flightMode.compare(guidedMode) != 0 && app->_isWorkThreadRunning)
                {
                    if (tryCount-- <= 0)
                    {
                        Logger::getInstance()->log(LOGLEVEL_ERROR, "Con't switch flight to \"" + guidedMode + "\"");
                        break;
                    }
                    app->_setFlightMode(guidedMode);
                    int checkCount = 10;
                    while (app->_flightMode.compare(guidedMode) != 0 && app->_isWorkThreadRunning && checkCount--)
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                }
            }
        } }, this, height);      // 创建线程，传递 Lambda 表达式和参数
    t.join();                              // 等待线程完成
}

void Application::_handleHeartbeat(const mavlink_message_t &msg)
{
    _receiveHeartbeat = true;
    _lastReceiveHeartbeatTimeMS = getCurrentMs();
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
        Logger::getInstance()->log(LOGLEVEL_INFO, "armed changed: " + std::string(_armed ? "armed" : "disArmed"));
    }

    if (_firmwareType != heartbeat.autopilot || _vehicleType != heartbeat.type) {
        _firmwareType = heartbeat.autopilot;
        _vehicleType = heartbeat.type;
        if (_flightModeInterface == nullptr) {
            _flightModeInterface = createFlightModeInterface();
        }
    }

    if (heartbeat.base_mode != _base_mode || heartbeat.custom_mode != _custom_mode) {
        _base_mode   = heartbeat.base_mode;
        _custom_mode = heartbeat.custom_mode;
        if (_flightModeInterface) {
            _flightMode = _flightModeInterface->flightMode(_base_mode, _custom_mode);
            Logger::getInstance()->log(LOGLEVEL_INFO, "flight mode changed: " + _flightMode);
        }
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
    // mavlink_position_target_local_ned_t positionTargetLocalNED;
    // mavlink_msg_position_target_local_ned_decode(&msg, &positionTargetLocalNED);
    // _acceleratedSpeed_x = positionTargetLocalNED.afx;
    // _acceleratedSpeed_y = positionTargetLocalNED.afy;
    // _acceleratedSpeed_z = positionTargetLocalNED.afz;

    // _velocityNED_x = positionTargetLocalNED.vx;
    // _velocityNED_y = positionTargetLocalNED.vy;
    // _velocityNED_z = positionTargetLocalNED.vz;
}

void Application::_handleHilStateQuateRnion(const mavlink_message_t &msg)
{
    mavlink_hil_state_quaternion_t hil;
    mavlink_msg_hil_state_quaternion_decode(&msg, &hil);
    _acceleratedSpeed_x = hil.xacc;
    _acceleratedSpeed_y = hil.yacc;
    _acceleratedSpeed_z = hil.zacc;
}

void Application::_handleHomePosition(const mavlink_message_t &msg)
{
    mavlink_home_position_t home;
    mavlink_msg_home_position_decode(&msg, &home);
    _homeLatitude = home.latitude / (double)1E7;
    _homeLongitude = home.longitude / (double)1E7;
    _homeAltitude = home.altitude / 1000.0;
    Logger::getInstance()->log(LOGLEVEL_DEBUG, "home position: lat=" + std::to_string(_homeLatitude) + " lng=" + std::to_string(_homeLongitude) + " alt=" + std::to_string(_homeAltitude));
}

void Application::_handleStatustext(const mavlink_message_t &msg)
{
    mavlink_statustext_t txt;
    mavlink_msg_statustext_decode(&msg, &txt);
    Logger::getInstance()->log(LOGLEVEL_INFO, "statusText:" + std::string(txt.text));
}

void Application::_handlePing(const mavlink_message_t &msg)
{
    mavlink_ping_t ping;
    mavlink_msg_ping_decode(&msg, &ping);

    if (_vehicleLink && _vehicleLink->isConnected())
    {
        mavlink_message_t msgPing;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {0};
        mavlink_msg_ping_pack(_systemId, _componentId, &msgPing, ping.time_usec, ping.seq, msg.sysid, msg.compid);
        int len = mavlink_msg_to_send_buffer(buf, &msgPing);
        _vehicleLink->writeData(buf, len);
    }
}

// TODO : 发送给指控的函数在下面添加

void Application::_sendPositionTOCtrlCenter()
{
    uint8_t buf[CTRL_CENTER_MSG_FRAME_LEN_MAX] = {0};
    ctrl_center_message_t message;
    ctrl_center_msg_position_pack(_longitude, _latitude, _altitude, message);
    int length = ctrl_center_msg_to_send_buffer(buf, message);
    if (_ctrlCenterLink) {
        _ctrlCenterLink->writeData(buf, length);
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
    if (_ctrlCenterLink) {
        _ctrlCenterLink->writeData(buf, length);
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
    if (_ctrlCenterLink) {
        _ctrlCenterLink->writeData(buf, length);
    }
}

void Application::_workThreadFunction(Application *app)
{
    auto now = std::chrono::high_resolution_clock::now();
    
    auto last_request_home_position = now;
    _isWorkThreadExited = false;
    while(_isWorkThreadRunning) {
        now = std::chrono::high_resolution_clock::now();

        // if (std::chrono::duration<float, std::milli>(now - last_request_home_position).count() > 5000)
        // {
        //     last_request_home_position = now;
        //     _requestHomePosition();
        // }

        if (!isHeartbeatLost() && _command_long.mode == 6)
        {
            double latitude = _command_long.position_1.latitude;
            double longitude = _command_long.position_1.longitude;
            double altitude = _command_long.position_1.altitude;
            float vx = _command_long.velocityNED_1.vx / 100.0f;
            float vy = _command_long.velocityNED_1.vy / 100.0f;
            float vz = _command_long.velocityNED_1.vz / 100.0f;
            float accx = _command_long.accSpeed_1.accx / 100.0f;
            float accy = _command_long.accSpeed_1.accy / 100.0f;
            float accz = _command_long.accSpeed_1.accz / 100.0f;
            float yaw = (_command_long.yaw_1 / 100.0f) * (M_PI/180.0f);
            float yawRate = (_command_long.yawAngleSpeed_1 / 100.0f) * (M_PI/180.0f);
            if (app->isMultiRotor()) {
                std::string guidedMode = app->_flightModeInterface->guidedMode();
                if (app->_flightMode.compare(guidedMode) == 0)
                {
                    // set_position_target_global_int
                    if (_vehicleLink && _vehicleLink->isConnected())
                    {
                        mavlink_message_t msg;
                        uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {0};
                        uint32_t bootTimeMs = getCurrentMs() - _bootTimeMs;
                        mavlink_msg_set_position_target_global_int_pack(_systemId, _componentId, &msg, bootTimeMs,
                                                                        _targetSystemId, _targetComponentId, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0,
                                                                        latitude * 1e7, longitude * 1e7, altitude - _homeAltitude, vx, vy, vz, accx, accy, accz, yaw, yawRate);
                        int len = mavlink_msg_to_send_buffer(buf, &msg);
                        _vehicleLink->writeData(buf, len);
                    }
#if 0 // TEST  原地起飞高度2米
                    if (_vehicleLink && _vehicleLink->isConnected())
                    {
                        mavlink_message_t msg;
                        uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {0};
                        uint32_t bootTimeMs = getCurrentMs() - _bootTimeMs;
                        mavlink_msg_set_position_target_global_int_pack(_systemId, _componentId, &msg, bootTimeMs,
                                                                        _targetSystemId, _targetComponentId, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0,
                                                                        _latitude * 1e7, _longitude * 1e7, 2, 0, 0, 0, 0, 0, 0, 0, 0);
                        int len = mavlink_msg_to_send_buffer(buf, &msg);
                        _vehicleLink->writeData(buf, len);
                    }
#endif
#if 0 //  TODO TEST  原地起飞高度2米
                    if (_vehicleLink && _vehicleLink->isConnected())
                    {
                        mavlink_message_t msg;
                        uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {0};
                        uint32_t bootTimeMs = getCurrentMs() - _bootTimeMs;
                        mavlink_msg_set_position_target_local_ned_pack(_systemId, _componentId, &msg, bootTimeMs,
                                                                       _targetSystemId, _targetComponentId, MAV_FRAME_LOCAL_NED, 0, 0, 0,
                                                                       2, 0, 0, 1, 0, 0, 1, 0, 0);
                        int len = mavlink_msg_to_send_buffer(buf, &msg);
                        _vehicleLink->writeData(buf, len);
                    }
#endif
                }
                
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (_flightModeInterface) {
        _setFlightMode(_flightModeInterface->pauseMode());
    }

    _isWorkThreadExited = true;
}

Application *Application::getInstance(void)
{
    return Application::_instance;
}