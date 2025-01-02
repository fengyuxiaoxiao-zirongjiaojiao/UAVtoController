/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 11:17:23
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2024-12-28 21:55:57
 * @FilePath: /UAVtoController/src/Protocol/ProtocolInterface.hPP
 * @Description: 
 */
#ifndef __PROTOCOL_INTERFACE_H_
#define __PROTOCOL_INTERFACE_H_

#include "mavlink.h"
#include "CtrlCenter.hpp"

class ProtocolInterface
{
private:
    /* data */
public:
    ProtocolInterface(/* args */){}
    ~ProtocolInterface(){}

    virtual void onDataReceive(const uint8_t *buf, int size) = 0;
};

class ProtocolObserver
{
public:
    virtual void onMavlinkMessageReceive(const mavlink_message_t &message) = 0;
    virtual void onCtrlCenterMessageReceive(const ctrl_center_message_t &message) = 0;
};

#endif // __PROTOCOL_INTERFACE_H_