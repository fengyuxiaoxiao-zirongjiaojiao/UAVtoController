#ifndef __PROTOCOL_CTRLCENTER_H_
#define __PROTOCOL_CTRLCENTER_H_

#include  "ProtocolInterface.hpp"
#include <map>
#include <string>

class ProtocolCtrlCenter : public ProtocolInterface
{
public:
    ProtocolCtrlCenter(ProtocolObserver *observer = nullptr);
    ~ProtocolCtrlCenter();

    virtual void onDataReceive(const uint8_t *buf, int size) override;
private:
    ProtocolObserver *_observer;
};

#endif // __PROTOCOL_CTRLCENTER_H_