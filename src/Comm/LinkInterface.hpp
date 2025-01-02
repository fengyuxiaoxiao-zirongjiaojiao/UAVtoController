/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 12:07:14
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2024-12-28 12:08:24
 * @FilePath: /UAVtoController/src/Comm/LinkInterface.hpp
 * @Description: 
 */
#ifndef __LINK_INTERFACE_H_
#define __LINK_INTERFACE_H_
#include <stdint.h>
class LinkInterface
{
public:
    LinkInterface(){}

    virtual bool connectLink() = 0;
    virtual void disconnectLink() = 0;
    virtual bool isConnected() = 0;
    virtual void writeData(const uint8_t *data, int length) = 0;
};

class LinkReadCallback
{
public:
    LinkReadCallback(/* args */){}
    
    virtual void onReadEvent(const uint8_t *buffer, int length) = 0;
};

#endif //__LINK_INTERFACE_H_