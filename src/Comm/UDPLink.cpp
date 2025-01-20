/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 22:59:57
 * @LastEditors: vincent_xjw@163.com
 * @LastEditTime: 2025-01-17 21:32:26
 * @FilePath: /UAVtoController/src/Comm/UDPLink.cpp
 * @Description: 
 */
#include "UDPLink.hpp"
#include "Application.hpp"

#define BUFFER_SIZE 256

std::mutex UDPLink::_mutex;

UDPLink::UDPLink(ProtocolInterface *protocol, LinkConfigure *config) : LinkInterface(config), _protocol(protocol)
{
}

UDPLink::~UDPLink()
{
    disconnectLink();
}

bool UDPLink::connectLink()
{
    // 启动线程
    _isThreadRunning = true;
    _readDataThread = std::thread(&UDPLink::_readDataFunction, this, this);
    _readDataThread.detach();
    return true;
}

void UDPLink::disconnectLink()
{
    _isThreadRunning = false;
    if (_readDataThread.joinable()) {
        _readDataThread.join();
    }
    Logger::getInstance()->log(LOGLEVEL_DEBUG, "");
}

bool UDPLink::isConnected()
{
    return (_isThreadRunning && _sockfd > 0);
}

void UDPLink::writeData(const uint8_t *data, int length)
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::ostringstream oss;
    ssize_t s = 0;
    if (isConnected()) {
        // 发送消息
        s = sendto(_sockfd, data, length, MSG_CONFIRM, (const struct sockaddr *)&_serverAddr, sizeof(_serverAddr));

        for (int i = 0; i < length; i++) {
            // 以两位16进制格式输出每个字符
            oss << std::hex << std::setw(2) << std::setfill('0') << (int)(data[i]) << " ";
        }
        Logger::getInstance()->log(LOGLEVEL_DEBUG, "客户端: 消息已发送"  + oss.str());
    }
}

void UDPLink::_readDataFunction(UDPLink *caller)
{
    // 创建套接字
    _sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (_sockfd < 0) {
        perror("客户端: 套接字创建失败");
        Logger::getInstance()->log(LOGLEVEL_FATAL, "客户端: 套接字创建失败");
        exit(EXIT_FAILURE);
    }

    // 设置客户端地址和端口（绑定自己的端口）
    if (_config->udpIsBindLocalPort()) {
        struct sockaddr_in server_addr, client_addr;
        memset(&client_addr, 0, sizeof(client_addr));
        client_addr.sin_family = AF_INET;
        client_addr.sin_addr.s_addr = INADDR_ANY; // 绑定到所有可用地址
        client_addr.sin_port = htons(_config->udpLocalPort());  // 指定客户端使用的端口号（例如 9090）

        if (bind(_sockfd, (const struct sockaddr *)&client_addr, sizeof(client_addr)) < 0) {
            perror("客户端: 绑定端口失败");
            close(_sockfd);
            exit(EXIT_FAILURE);
        }
    }
    

    // 设置服务器地址和端口
    memset(&_serverAddr, 0, sizeof(_serverAddr));
    _serverAddr.sin_family = AF_INET;
    _serverAddr.sin_port = htons(_config->udpServerPort());
    if (inet_pton(AF_INET, _config->udpServerIP().c_str(), &_serverAddr.sin_addr) <= 0)
    {
        perror("客户端: 无效地址或地址不支持");
        Logger::getInstance()->log(LOGLEVEL_FATAL, "客户端: 无效地址或地址不支持 " + std::string("IP:") + _config->udpServerIP() + std::string(" port:") + std::to_string(_config->udpServerPort()));
        close(_sockfd);
        exit(EXIT_FAILURE);
    }

    // // 发送消息
    // std::string message = "Hello from Client!";
    // writeData((const uint8_t*)message.c_str(), message.length());
    // std::cout << "客户端: 消息已发送" << std::endl;

    // 接收服务器回复
    socklen_t server_len = sizeof(_serverAddr);
    uint8_t buffer[BUFFER_SIZE];
    while(_isThreadRunning) {
        int n = recvfrom(_sockfd, buffer, BUFFER_SIZE, MSG_WAITALL, (struct sockaddr *)&_serverAddr, &server_len);
        // buffer[n] = '\0';
        // std::cout << "客户端: 收到回复: " << buffer << std::endl;
        if (_protocol) {
            _protocol->onDataReceive(buffer, n);
        }
    }

    close(_sockfd);
}