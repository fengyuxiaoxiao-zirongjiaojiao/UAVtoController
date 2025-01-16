/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 10:40:08
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2025-01-16 11:46:59
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
, _serialLink(nullptr)
, _ctrlCenter(nullptr)
, _udpLink(nullptr)
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
    _argSerialPort = reader.GetString("COMM","SerialPort", "/dev/ttyS0");
    _argBaudRate = reader.GetInteger("COMM","SerialRate", 115200);
    _argUdpServerIP = reader.GetString("COMM","UdpHost", "127.0.0.1");
    _argUdpServerPort = reader.GetInteger("COMM","UdpPort", 2001);
    int level = reader.GetInteger("LOG","LEVEL", 1);
    Logger::getInstance()->setLogLevel((LogLevel)level);

    Logger::getInstance()->log(LOGLEVEL_INFO, "SerialPort:" + _argSerialPort + " SerialRate:" + std::to_string(_argBaudRate) + 
                                                " UdpHost:" + _argUdpServerIP + " UdpPort:" + std::to_string(_argUdpServerPort) + 
                                                " LogLevel:" + std::to_string(level));
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
    iniFile << "[COMM]" << std::endl;
    iniFile << "SerialPort = " << _argSerialPort << std::endl;
    iniFile << "SerialRate = " << _argBaudRate << std::endl;
    iniFile << "UdpHost = " << _argUdpServerIP << std::endl;
    iniFile << "UdpPort = " << _argUdpServerPort << std::endl;

    iniFile << "[LOG]" << std::endl;
    iniFile << "LEVEL = " << Logger::getInstance()->logLevel() << std::endl;

    iniFile.close();
    std::cout << "配置已更新并写回到INI文件!" << std::endl;
}

bool Application::init()
{
    loadSettings();

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
    saveSettings();

    _quit = true;
    _isWorkThreadRunning = false;
    if (_workThread.joinable()) {
        _workThread.join();
    }
    while(!_isWorkThreadExited) { std::this_thread::sleep_for(std::chrono::milliseconds(200)); }

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

    _isWorkThreadRunning = true;
    _workThread = std::thread(&Application::_workThreadFunction, this, this);
    _workThread.detach();

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
    default:
        break;
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
        _command_long = command_long;
        // TODO : 收到指控的数据后进行相应的处理
        if (command_long.mode == 0) {// 0-待机 
            // TODO
        } else if (command_long.mode == 1) {// 1-作战 
            // TODO
        } else if (command_long.mode == 2) {// 2-悬停
            if (_flightModeInterface) {
                _setFlightMode(_flightModeInterface->loiterMode());
            }
        } else if (command_long.mode == 3) {// 3-一键起飞 
            _guidedTakeoff(10);
        } else if (command_long.mode == 4) {// 4-一键返航 
            if (_flightModeInterface) {
                _setFlightMode(_flightModeInterface->rtlMode());
            }
        } else if (command_long.mode == 5) {// 5-原地降落 
            if (_flightModeInterface) {
                _setFlightMode(_flightModeInterface->landMode());
            }
        } else if (command_long.mode == 6) {// 6-自主巡航 
            // _workThreadFunction();
        } else if (command_long.mode == 7) {// 7-定高，需要发送高度信息
            if (_flightModeInterface) {
                _setFlightMode(_flightModeInterface->hoverMode());
            }
        } else if (command_long.mode == 8) {// 8-遥操作模式
            if (_flightModeInterface) {
                _setFlightMode(_flightModeInterface->stabilizedMode());
            }
        }

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

void Application::_sendHeartbeat()
{
    if (_serialLink && _serialLink->isConnected()) {
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

    if (_flightModeInterface && _flightModeInterface->setFlightMode(mode, base_mode, custom_mode)) {
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

void Application::_sendMavCommand(MAV_CMD command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    if (_serialLink && _serialLink->isConnected()) {
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {0};
        mavlink_msg_command_long_pack(255, MAV_COMP_ID_MISSIONPLANNER, &msg, _targetSystemId, _targetComponentId, command, 0,
                                        param1, param2, param3, param4, param5, param6, param7);
        int len = mavlink_msg_to_send_buffer(buf, &msg);
        _serialLink->writeData(buf, len);
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
                while(!app->armed()) {
                    app->setArmed(true);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                app->_sendMavCommand(MAV_CMD_NAV_TAKEOFF,
                        -1,                                     // No pitch requested
                        0, 0,                                   // param 2-4 unused
                        NAN, NAN, NAN,                          // No yaw, lat, lon
                        alt);
            } else if (app->isPX4FirmwareClass()) {
                app->_sendMavCommand(MAV_CMD_NAV_TAKEOFF,
                        -1,                                     // No pitch requested
                        0, 0,                                   // param 2-4 unused
                        NAN, NAN, NAN,                          // No yaw, lat, lon
                        alt);
            }
        }
    }, this, relAltitude); // 创建线程，传递 Lambda 表达式和参数
    t.join(); // 等待线程完成
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
#if 1//TEST 测试
    sysStatus.position.altitude = 1000000000000000000;
    sysStatus.position.latitude = 1000000000000000000;
    sysStatus.position.longitude = 1000000000000000000;
    sysStatus.rtkFixType = 125;
    sysStatus.relativeAltitude = 1000000000000000000;
    sysStatus.velocity.vx = -10000;
    sysStatus.velocity.vy = -10000;
    sysStatus.velocity.vz = -10000;
    sysStatus.accSpeed.accx = -10000;
    sysStatus.accSpeed.accy = -10000;
    sysStatus.accSpeed.accz = -10000;
    sysStatus.roll = -10000;
    sysStatus.pitch = -10000;
    sysStatus.yaw = 20000;
    sysStatus.angleVelocity.vx = -10000;
    sysStatus.angleVelocity.vy = -10000;
    sysStatus.angleVelocity.vz = -10000;
    sysStatus.allSensorsHealthy = 125;
    sysStatus.commandAck = 125;
    memset(&sysStatus.reserved[0], 125, 32);
#else
    sysStatus.position.altitude = _altitude;
    sysStatus.position.latitude = _latitude;
    sysStatus.position.longitude = _longitude;
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
    memset(&sysStatus.reserved[0], 0, 32);
#endif
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

void Application::_workThreadFunction(Application *app)
{
    _isWorkThreadExited = false;
    while(_isWorkThreadRunning) {
        if (!isHeartbeatLost() && _command_long.mode == 6) {
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
                while(app->_flightMode.compare(guidedMode) != 0) {
                    app->_setFlightMode(guidedMode);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                while(!app->armed()) {
                    app->setArmed(true);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                while(app->_relativeAltitude < 10) {
                    app->_sendMavCommand(MAV_CMD_NAV_TAKEOFF,
                        -1,                                     // No pitch requested
                        0, 0,                                   // param 2-4 unused
                        NAN, NAN, NAN,                          // No yaw, lat, lon
                        10);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                
                // set_position_target_global_int
                if (_serialLink && _serialLink->isConnected()) {
                    mavlink_message_t msg;
                    uint8_t buf[MAVLINK_MAX_PACKET_LEN] = {0};
                    uint32_t bootTimeMs = getCurrentMs() - _bootTimeMs;
                    mavlink_msg_set_position_target_global_int_pack(255, MAV_COMP_ID_MISSIONPLANNER, &msg, bootTimeMs, 
                        _targetSystemId, _targetComponentId, MAV_FRAME_GLOBAL_INT, 0,
                        latitude * 1e7, longitude * 1e7, altitude, vx, vy, vz, accx, accy, accz, yaw, yawRate);
                    int len = mavlink_msg_to_send_buffer(buf, &msg);
                    _serialLink->writeData(buf, len);
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