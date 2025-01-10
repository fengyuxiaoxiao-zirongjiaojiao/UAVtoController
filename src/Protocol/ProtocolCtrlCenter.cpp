/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 21:28:29
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2025-01-05 15:38:25
 * @FilePath: /UAVtoController/src/Protocol/ProtocolCtrlCenter.cpp
 * @Description: 
 */
#include "ProtocolCtrlCenter.hpp"
#include <iostream>
#include "Logger.hpp"

ProtocolCtrlCenter::ProtocolCtrlCenter(ProtocolObserver *observer) : _observer(observer)
{
}

ProtocolCtrlCenter::~ProtocolCtrlCenter()
{
}

void ProtocolCtrlCenter::onDataReceive(const uint8_t *buf, int size)
{
    Logger::getInstance()->log(LOGLEVEL_DEBUG, "Recevice size: " + std::to_string(size) + " from CtrlCenter." );
    std::ostringstream oss;

    ctrl_center_message_t message;
    ctrl_center_status_t status;
    memset(&message, 0, sizeof(ctrl_center_message_t));
    memset(&status, 0, sizeof(ctrl_center_status_t));
    // TODO: 解析数据
    for (int i = 0; i < size; i++) {
        // printf("%02x ", buf[i]);
        // 以两位16进制格式输出每个字符
        oss << std::hex << std::setw(2) << std::setfill('0') << (int)(buf[i]) << " ";
        if (ctrl_center_parse_char(buf[i], &message, status)) {
            if (_observer) {
                _observer->onCtrlCenterMessageReceive(message);
            }
        }
    }
    Logger::getInstance()->log(LOGLEVEL_DEBUG, "Recevice from CtrlCenter: " + oss.str() );
}