/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 21:28:29
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2024-12-29 14:58:52
 * @FilePath: /UAVtoController/src/Protocol/ProtocolCtrlCenter.cpp
 * @Description: 
 */
#include "ProtocolCtrlCenter.hpp"
#include <iostream>

ProtocolCtrlCenter::ProtocolCtrlCenter(ProtocolObserver *observer) : _observer(observer)
{
}

ProtocolCtrlCenter::~ProtocolCtrlCenter()
{
}

void ProtocolCtrlCenter::onDataReceive(const uint8_t *buf, int size)
{
    std::cout << __func__ << " " << __LINE__ << size << std::endl;
    ctrl_center_message_t message;
    ctrl_center_status_t status;
    memset(&message, 0, sizeof(ctrl_center_message_t));
    memset(&status, 0, sizeof(ctrl_center_status_t));
    // TODO: 解析数据
    for (int i = 0; i < size; i++) {
        printf("%02x ", buf[i]);
        if (ctrl_center_parse_char(buf[i], &message, status)) {
            if (_observer) {
                _observer->onCtrlCenterMessageReceive(message);
            }
        }
    }
    printf("\n");
    
}