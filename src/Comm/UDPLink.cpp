#include "UDPLink.hpp"
#include "Application.hpp"

#define BUFFER_SIZE 256

std::mutex UDPLink::_mutex;

UDPLink::UDPLink(ProtocolInterface *protocol) : _protocol(protocol)
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
}

bool UDPLink::isConnected()
{
    return (_isThreadRunning && _sockfd > 0);
}

void UDPLink::writeData(const uint8_t *data, int length)
{
    std::lock_guard<std::mutex> lock(_mutex);
    ssize_t s = 0;
    if (isConnected()) {
        // 发送消息
        std::string message = "Hello from Client!";
        s = sendto(_sockfd, data, length, MSG_CONFIRM, (const struct sockaddr *)&_serverAddr, sizeof(_serverAddr));
        std::cout << "客户端: 消息已发送" << s << std::endl;
    }
}

void UDPLink::_readDataFunction(UDPLink *caller)
{
    // 创建套接字
    _sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (_sockfd < 0) {
        perror("客户端: 套接字创建失败");
        exit(EXIT_FAILURE);
    }

    // 设置服务器地址和端口
    memset(&_serverAddr, 0, sizeof(_serverAddr));
    _serverAddr.sin_family = AF_INET;
    _serverAddr.sin_port = htons(Application::getInstance()->argUdpServerPort());
    if (inet_pton(AF_INET, Application::getInstance()->argUdpServerIP().c_str(), &_serverAddr.sin_addr) <= 0) {
        perror("客户端: 无效地址或地址不支持");
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