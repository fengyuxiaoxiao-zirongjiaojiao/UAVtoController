/*
 * @Description:
 * @Author: vincent_xjw@163.com
 * @Date: 2023-10-13 11:51:56
 * @LastEditTime: 2023-10-13 15:11:27
 * @LastEditors: vincent_xjw@163.com
 */
#include <iostream>
#include <vector>
#include <string>
#include <thread>

#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"

class TestSerial : public itas109::CSerialPortListener
{
private:
    /* data */
public:
    TestSerial(/* args */)
    {
        m_SerialPort.connectReadEvent(this);
    }
    ~TestSerial()
    {
    }

    void open(const std::string &portName, int baudrate = 57600, itas109::Parity parity = itas109::ParityNone, itas109::DataBits dataBits = itas109::DataBits8, itas109::StopBits stopBits = itas109::StopOne)
    {
        m_SerialPort.init(portName.c_str(),
                          baudrate,
                          parity,
                          dataBits,
                          stopBits);

        m_SerialPort.setReadIntervalTimeout(0);

        if (m_SerialPort.open())
        {
            std::cout << "serial open success[" << portName << "]" << std::endl;
        }
        else
        {
            std::cout << "serial open failed[" << portName << "] " << m_SerialPort.getLastError() << " - " << m_SerialPort.getLastErrorMsg() << std::endl;
        }
    }

    void close() { m_SerialPort.close(); }

    void send(const std::string &str)
    {
        if (m_SerialPort.isOpen())
        {
            const char *s = str.c_str();

            // 支持中文并获取正确的长度
            m_SerialPort.writeData(s, str.length());

            tx += str.length();
        }
    }

    void onReadEvent(const char *portName, unsigned int readBufferLen)
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
                std::cout << "recv: portName=" << std::string(portName) << " length:" << recLen << " data:" << std::string(str) << std::endl;
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

private:
    itas109::CSerialPort m_SerialPort;

    int tx;
    int rx;
};

int main(int argc, char **argv)
{
    std::cout << "serial port test..." << std::endl;
    int index = -1;
    std::vector<itas109::SerialPortInfo> portNameList = itas109::CSerialPortInfo::availablePortInfos();

    for (size_t i = 0; i < portNameList.size(); i++)
    {
        std::cout << std::string(portNameList[i].portName) << std::endl;
        if (argc > 1 && std::string(argv[1]) == std::string(portNameList[i].portName)) {
            index = i;
        }
    }
    if (index == -1 && portNameList.size() > 0) {
        index = 0;
    }
    if (index == -1) {
        printf("Error not fond device.\n");
        return 0;
    }

    TestSerial serial;
    serial.open(std::string(portNameList[index].portName));

    serial.send("hello world");
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}