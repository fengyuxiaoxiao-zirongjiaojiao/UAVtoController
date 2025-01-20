/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 22:59:49
 * @LastEditors: vincent_xjw@163.com
 * @LastEditTime: 2025-01-17 20:38:09
 * @FilePath: /UAVtoController/src/Comm/UDPLink.hpp
 * @Description: 
 */
#ifndef __UDP_LINK_H_
#define __UDP_LINK_H_
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <arpa/inet.h>
#include <unistd.h>
#include "ProtocolInterface.hpp"
#include "LinkInterface.hpp"

class UDPLink : public LinkInterface
{
public:
    UDPLink(ProtocolInterface *protocol, LinkConfigure *config);
    ~UDPLink();
    
    virtual bool connectLink() override;
    virtual void disconnectLink() override;
    virtual bool isConnected() override;
    virtual void writeData(const uint8_t *data, int length) override;
private:
    ProtocolInterface *_protocol;

    void _readDataFunction(UDPLink *caller);
    std::mutex _readDataMutex;
    std::thread _readDataThread;
    bool _isThreadRunning = false;

    // UDP
    int _sockfd;
    struct sockaddr_in _serverAddr;

    static std::mutex _mutex;
};

#endif // __UDP_LINK_H_