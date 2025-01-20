/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 12:09:37
 * @LastEditors: vincent_xjw@163.com
 * @LastEditTime: 2025-01-17 20:36:49
 * @FilePath: /UAVtoController/src/Comm/SerialLink.hpp
 * @Description: 
 */
#ifndef __SERIALLINK_H_
#define __SERIALLINK_H_
#include <string>
#include <mutex>
#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"
#include "ProtocolInterface.hpp"
#include "LinkInterface.hpp"

class SerialLink : public itas109::CSerialPortListener , public LinkInterface
{
public:
    SerialLink(ProtocolInterface *protocol, LinkConfigure *config);
    ~SerialLink();

    bool open(const std::string &portName, int baudrate = 57600, itas109::Parity parity = itas109::ParityNone, itas109::DataBits dataBits = itas109::DataBits8, itas109::StopBits stopBits = itas109::StopOne);
    void send(const std::string &str);
    void onReadEvent(const char *portName, unsigned int readBufferLen);

    virtual bool connectLink() override;
    virtual void disconnectLink() override;
    virtual bool isConnected() override;
    virtual void writeData(const uint8_t *data, int length) override;
private:
    itas109::CSerialPort m_SerialPort;

    int tx;
    int rx;

    ProtocolInterface *_protocol;
    static std::mutex _mutex;
};

#endif 