/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 11:20:51
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2025-01-05 15:40:28
 * @FilePath: /UAVtoController/src/Protocol/ProtocolMavlink.cpp
 * @Description: 
 */
#include "ProtocolMavlink.hpp"
#include <iostream>
#include "Logger.hpp"

ProtocolMavlink::ProtocolMavlink(ProtocolObserver *observer) : _observer(observer)
{
}

ProtocolMavlink::~ProtocolMavlink()
{
}

void ProtocolMavlink::onDataReceive(const uint8_t *buf, int size)
{
    Logger::getInstance()->log(LOGLEVEL_DEBUG, "Recevice size: " + std::to_string(size) + " from Vehicle." );
    if (size > 0)
    {
        // Something received - print out all bytes and parse packet
        mavlink_message_t msg;
        mavlink_status_t status;
        memset(&msg, 0, sizeof(mavlink_message_t));
        memset(&status, 0, sizeof(mavlink_status_t));

        std::ostringstream oss;
        for (int i = 0; i < size; ++i)
        {
            oss << std::hex << std::setw(2) << std::setfill('0') << (int)(buf[i]) << " ";
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
            {
                // Packet received
                // printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
                if (_observer) {
                    _observer->onMavlinkMessageReceive(msg);
                }
            }
        }
        Logger::getInstance()->log(LOGLEVEL_DEBUG, "Recevice from Vehicle: " + oss.str() );
    }
}