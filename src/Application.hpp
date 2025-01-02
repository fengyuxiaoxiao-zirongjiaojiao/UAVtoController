/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 10:39:34
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2025-01-02 15:28:48
 * @FilePath: /UAVtoController/src/Application.hpp
 * @Description: 
 */
#ifndef __APPLICATION_H_
#define __APPLICATION_H_
#include <memory>
#include "SerialLink.hpp"
#include "ProtocolMavlink.hpp"
#include "UDPLink.hpp"
#include "ProtocolCtrlCenter.hpp"

class Application : public ProtocolObserver
{
public:
    Application(int argc, char **argv);
    ~Application();

    static Application *_instance;
    static Application *getInstance(void);

    bool init();
    void quit();
    void exec();

    /**
     * 接收来自飞机的数据
    */
    virtual void onMavlinkMessageReceive(const mavlink_message_t &message) override;
    /**
     * 接收来自指控的数据
    */
    virtual void onCtrlCenterMessageReceive(const ctrl_center_message_t &message) override;

    std::string argUdpServerIP() { return _argUdpServerIP; }
    int argUdpServerPort() { return _argUdpServerPort; }
    int argUdpLocalPort() { return _argUdpLocalPort; }
private:
    // 飞机 mavlink
    void _sendHeartbeat();
    void _setFlightMode(const std::string &mode);
    void _requestDataStream(MAV_DATA_STREAM stream, uint16_t rate);
    void _handleHeartbeat(const mavlink_message_t &msg);
    void _handleSysStatus(const mavlink_message_t &msg);
    void _handleLocalPositionNED(const mavlink_message_t &msg);
    void _handleRawIMU(const mavlink_message_t &msg);
    void _handlePositionTargetLocalNED(const mavlink_message_t &msg);
    // 指控
    void _sendPositionTOCtrlCenter();
    void _sendSysStatusTOCtrlCenter();
    void _sendBatteryInfoToCtrlCenter();
private:
    bool _quit = false;

    SerialLink *_serialLink;
    std::string _argSerialPort = "/dev/ttyS0";
    int _argBaudRate = 115200;
    
    ProtocolMavlink *_mavlink;
    uint8_t _targetSystemId;
    uint8_t _targetComponentId;
    bool _armed = false;
    uint8_t _base_mode = 0;     ///< base_mode from HEARTBEAT
    uint32_t _custom_mode = 0;
    uint8_t       _firmwareType; // MAV_AUTOPILOT
    uint8_t            _vehicleType; //MAV_TYPE
    FlightModeDecode _flightModeDecode;
    std::string _flightMode = "Unknown"; // 模式
    bool _receiveHeartbeat = false;
    float _voltageBattery = 0;// 电压
    float _currentBattery = 0;// 电流A
    int _batteryRemaining = 0;// 剩余电量
    int _allSensorsHealthy = -1; // 0 表示健康; -1表示无效状态；其他为状态码
    uint8_t _commandAck;
    // 位置信息
    double _latitude = 0; // 纬度
    double _longitude = 0; // 经度
    float _altitude = 0; // 绝对高度
    float _relativeAltitude = 0; // 相对高度
    // 姿态信息
    float _pitch = 0; // 俯仰
    float _roll = 0;  // 横滚
    float _yaw = 0;   // 航向
    // rtk状态
    int _rtkFixType = 0;
    // 运动速度 北东地
    float _velocityNED_x = 0;
    float _velocityNED_y = 0;
    float _velocityNED_z = 0;
    // 运动加速度
    float _acceleratedSpeed_x = 0;
    float _acceleratedSpeed_y = 0;
    float _acceleratedSpeed_z = 0;
    // 角速度
    float _angleSpeed_x = 0;
    float _angleSpeed_y = 0;
    float _angleSpeed_z = 0;


    // 指控
    UDPLink *_udpLink;
    std::string _argUdpServerIP = "192.168.0.101";//"127.0.0.1";
    int _argUdpServerPort = 2001;
    int _argUdpLocalPort = 2002;

    ProtocolCtrlCenter *_ctrlCenter;
};



#endif // __APPLICATION_H_