#include "SerialLink.hpp"
#include <iostream>
#include "Logger.hpp"

std::mutex SerialLink::_mutex;

SerialLink::SerialLink(ProtocolInterface *protocol, LinkConfigure *config) : LinkInterface(config), _protocol(protocol)
{
    Logger::getInstance()->log(LOGLEVEL_DEBUG, "");
    m_SerialPort.connectReadEvent(this);
}

SerialLink::~SerialLink()
{
    Logger::getInstance()->log(LOGLEVEL_DEBUG, "");
    _protocol = nullptr;
    disconnectLink();
}

bool SerialLink::open(const std::string &portName, int baudrate, itas109::Parity parity, itas109::DataBits dataBits, itas109::StopBits stopBits)
{
    m_SerialPort.init(portName.c_str(),
                        baudrate,
                        parity,
                        dataBits,
                        stopBits);

    m_SerialPort.setReadIntervalTimeout(0);

    if (m_SerialPort.open())
    {
        Logger::getInstance()->log(LOGLEVEL_INFO, "Serial open success[" + portName + "] rate:" + std::to_string(baudrate));
        return true;
    }
    Logger::getInstance()->log(LOGLEVEL_ERROR, "Serial open failed[" + portName + "] " + std::to_string(m_SerialPort.getLastError()) + " - " + m_SerialPort.getLastErrorMsg());
    return false;
}

void SerialLink::send(const std::string &str)
{
    if (m_SerialPort.isOpen())
    {
        const char *s = str.c_str();

        // 支持中文并获取正确的长度
        m_SerialPort.writeData(s, str.length());

        tx += str.length();
    }
}

void SerialLink::onReadEvent(const char *portName, unsigned int readBufferLen)
{
    if (readBufferLen > 0)
    {
        int recLen = 0;
        char *str = NULL;
        str = new char[readBufferLen];
        recLen = m_SerialPort.readData(str, readBufferLen);

        if (recLen > 0)
        {
            // TODO: 中文需要由两个字符拼接，否则显示为空""
            // std::cout << "recv: portName=" << std::string(portName) << " length:" << recLen << " data:" << std::string(str) << std::endl;
            // std::cout << "recv: portName=" << std::string(portName) << " length:" << recLen << " data:" << globalFunc::stringToHex((unsigned char*)str, recLen) << std::endl;
            
            if (_protocol) {
                _protocol->onDataReceive((uint8_t*)str, recLen);
            }
        }
        else
        {
        }

        if (str)
        {
            delete[] str;
            str = NULL;
        }
    }
}

bool SerialLink::connectLink()
{
    return open(_config->serialPort(), _config->serialBaudRate());
}

void SerialLink::disconnectLink()
{
    m_SerialPort.close();
}

bool SerialLink::isConnected()
{
    return m_SerialPort.isOpen();
}

void SerialLink::writeData(const uint8_t *data, int length)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (isConnected()) {
        m_SerialPort.writeData(data, length);

        tx += length;
    }
}