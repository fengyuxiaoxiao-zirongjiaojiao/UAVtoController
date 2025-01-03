/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 11:20:39
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2025-01-03 13:44:33
 * @FilePath: /UAVtoController/src/Protocol/ProtocolMavlink.hpp
 * @Description: 
 */
#ifndef __PROTOCOL_MAVLINK_H_
#define __PROTOCOL_MAVLINK_H_
#include  "ProtocolInterface.hpp"
#include <map>
#include <vector>
#include <string>

class ProtocolMavlink : public ProtocolInterface
{
private:
    /* data */
public:
    ProtocolMavlink(ProtocolObserver *observer = nullptr);
    ~ProtocolMavlink();

    virtual void onDataReceive(const uint8_t *buf, int size) override;
private:
    ProtocolObserver *_observer;
};




#endif //__PROTOCOL_MAVLINK_H_