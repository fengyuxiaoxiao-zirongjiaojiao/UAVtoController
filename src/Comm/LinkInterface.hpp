/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 12:07:14
 * @LastEditors: vincent_xjw@163.com
 * @LastEditTime: 2025-01-17 21:37:05
 * @FilePath: /UAVtoController/src/Comm/LinkInterface.hpp
 * @Description: 
 */
#ifndef __LINK_INTERFACE_H_
#define __LINK_INTERFACE_H_
#include <stdint.h>
#include <sstream>
#include <iostream>
class LinkConfigure;

class LinkInterface
{
public:
    LinkInterface(LinkConfigure *config) : _config(config) {}
    virtual ~LinkInterface() {}

    virtual bool connectLink() = 0;
    virtual void disconnectLink() = 0;
    virtual bool isConnected() = 0;
    virtual void writeData(const uint8_t *data, int length) = 0;

    LinkConfigure *_config;
};

class LinkReadCallback
{
public:
    LinkReadCallback(/* args */){}
    
    virtual void onReadEvent(const uint8_t *buffer, int length) = 0;
};

class LinkConfigure
{
public:
    LinkConfigure() {}

    std::string type() { return _type; }
    void setType(const std::string &type) { _type = type; }

    std::string serialPort() { return _serialPort; }
    void setSerialPort(const std::string &port) { _serialPort = port; }

    int serialBaudRate () { return _serialBaudRate; }
    void setSerialBaudrate(int baudrate) { _serialBaudRate = baudrate; }


    std::string udpServerIP() { return _udpServerIP; }
    void setUdpServerIP(const std::string &ip) { _udpServerIP = ip; }

    int udpServerPort() { return _udpServerPort; }
    void setUdpServerPort(int port) { _udpServerPort = port; }

    int udpLocalPort() { return _udpLocalPort; }
    void setUdpLocalPort(int port) { _udpLocalPort = port; }

    bool udpIsBindLocalPort() { return _udpIsBindLocalPort; }
    void setUdpIsBindLocalPort(bool bind) { _udpIsBindLocalPort = bind; }

    std::string toString() {
        std::ostringstream oss;
        oss << "Type = " << _type << std::endl;
        oss << "SerialPort = " << _serialPort << std::endl;
        oss << "SerialRate = " << std::to_string(_serialBaudRate) << std::endl;
        oss << "UdpHost = " << _udpServerIP << std::endl;
        oss << "UdpPort = " << std::to_string(_udpServerPort) << std::endl;
        oss << "UdpLocalPort = " << std::to_string(_udpLocalPort) << std::endl;
        oss << "IsBindLocalPort = " << std::string(_udpIsBindLocalPort ? "1" : "0") << std::endl;

        // std::string str = "Type = " + _type + "\nSerialPort = " + _serialPort + "\nSerialRate = " + std::to_string(_serialBaudRate);
        // str += "\nUdpHost = " + _udpServerIP + "\nUdpPort = " + std::to_string(_udpServerPort) + "\nUdpLocalPort = " + std::to_string(_udpLocalPort) + "\nIsBindLocalPort = " + std::string(_udpIsBindLocalPort ? "1" : "0");
        // return str;
        return oss.str();
    }

private: 
    std::string _type = "serial";

    std::string _serialPort = "/dev/ttyS0";
    int _serialBaudRate = 115200;

    std::string _udpServerIP = "127.0.0.1";
    int _udpServerPort = 2001;
    int _udpLocalPort = 2002;
    bool _udpIsBindLocalPort = true; // 是否绑定端口号
};
#endif //__LINK_INTERFACE_H_