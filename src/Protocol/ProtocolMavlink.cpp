/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 11:20:51
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2024-12-29 14:55:57
 * @FilePath: /UAVtoController/src/Protocol/ProtocolMavlink.cpp
 * @Description: 
 */
#include "ProtocolMavlink.hpp"
#include <iostream>

ProtocolMavlink::ProtocolMavlink(ProtocolObserver *observer) : _observer(observer)
{
}

ProtocolMavlink::~ProtocolMavlink()
{
}

void ProtocolMavlink::onDataReceive(const uint8_t *buf, int size)
{
    // std::cout << "size:" << size << std::endl;
    if (size > 0)
    {
        // Something received - print out all bytes and parse packet
        mavlink_message_t msg;
        mavlink_status_t status;
        memset(&msg, 0, sizeof(mavlink_message_t));
        memset(&status, 0, sizeof(mavlink_status_t));
        printf("Bytes Received: %d\nDatagram: ", (int)size);
        for (int i = 0; i < size; ++i)
        {
            char temp = buf[i];
            printf("%02x ", (unsigned char)temp);
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
            {
                // Packet received
                printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
                if (_observer) {
                    _observer->onMavlinkMessageReceive(msg);
                }
            }
        }
        printf("\n");
    }
}