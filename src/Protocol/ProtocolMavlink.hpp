/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 11:20:39
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2024-12-28 14:33:43
 * @FilePath: /UAVtoController/src/Protocol/ProtocolMavlink.hpp
 * @Description: 
 */
#ifndef __PROTOCOL_MAVLINK_H_
#define __PROTOCOL_MAVLINK_H_
#include  "ProtocolInterface.hpp"
#include <map>
#include <string>

class FlightModeDecode {
public:
    enum APMCopterMode {
        STABILIZE   = 0,   // hold level position
        ACRO        = 1,   // rate control
        ALT_HOLD    = 2,   // AUTO control
        AUTO        = 3,   // AUTO control
        GUIDED      = 4,   // AUTO control
        LOITER      = 5,   // Hold a single location
        RTL         = 6,   // AUTO control
        CIRCLE      = 7,   // AUTO control
        POSITION    = 8,   // Deprecated
        LAND        = 9,   // AUTO control
        OF_LOITER   = 10,  // Deprecated
        DRIFT       = 11,  // Drift 'Car Like' mode
        RESERVED_12 = 12,  // RESERVED FOR FUTURE USE
        SPORT       = 13,
        FLIP        = 14,
        AUTOTUNE    = 15,
        POS_HOLD    = 16, // HYBRID LOITER.
        BRAKE       = 17,
        THROW       = 18,
        AVOID_ADSB  = 19,
        GUIDED_NOGPS= 20,
        SMART_RTL   = 21,  // SMART_RTL returns to home by retracing its steps
        FLOWHOLD    = 22,  // FLOWHOLD holds position with optical flow without rangefinder
#if 0
    // Follow me not ready for Stable
        FOLLOW      = 23,  // follow attempts to follow another vehicle or ground station
#endif
        ZIGZAG      = 24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
    };

    FlightModeDecode(){}
    void init(uint8_t firmwareType,uint8_t vehicleType) 
    {
        _firmwareType = firmwareType;
        _vehicleType = vehicleType;

        _modeString.insert(std::pair < int,std::string > (STABILIZE,    "Stabilize"));
        _modeString.insert(std::pair < int,std::string > (ACRO,         "Acro"));
        _modeString.insert(std::pair < int,std::string > (ALT_HOLD,     "Altitude Hold"));
        _modeString.insert(std::pair < int,std::string > (AUTO,         "Auto"));
        _modeString.insert(std::pair < int,std::string > (GUIDED,       "Guided"));
        _modeString.insert(std::pair < int,std::string > (LOITER,       "Loiter"));
        _modeString.insert(std::pair < int,std::string > (RTL,          "RTL"));
        _modeString.insert(std::pair < int,std::string > (CIRCLE,       "Circle"));
        _modeString.insert(std::pair < int,std::string > (LAND,         "Land"));
        _modeString.insert(std::pair < int,std::string > (DRIFT,        "Drift"));
        _modeString.insert(std::pair < int,std::string > (SPORT,        "Sport"));
        _modeString.insert(std::pair < int,std::string > (FLIP,         "Flip"));
        _modeString.insert(std::pair < int,std::string > (AUTOTUNE,     "Autotune"));
        _modeString.insert(std::pair < int,std::string > (POS_HOLD,     "Position Hold"));
        _modeString.insert(std::pair < int,std::string > (BRAKE,        "Brake"));
        _modeString.insert(std::pair < int,std::string > (THROW,        "Throw"));
        _modeString.insert(std::pair < int,std::string > (AVOID_ADSB,   "Avoid ADSB"));
        _modeString.insert(std::pair < int,std::string > (GUIDED_NOGPS, "Guided No GPS"));
        _modeString.insert(std::pair < int,std::string > (SMART_RTL,    "Smart RTL"));
        _modeString.insert(std::pair < int,std::string > (FLOWHOLD,     "Flow Hold" ));
        _modeString.insert(std::pair < int,std::string > (ZIGZAG,       "ZigZag" ));
    }

    std::string flightMode(uint8_t baseMode, uint32_t customMode) {
        std::string flightMode = "Unknown";

        if (baseMode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
            std::map< int, std::string >::reverse_iterator iter; 
            for(iter = _modeString.rbegin(); iter != _modeString.rend(); iter++) {
                if (iter->first == customMode) {
                    flightMode = iter->second;
                }
            }
        }
        return flightMode;
    }

    bool setFlightMode(const std::string flightMode, uint8_t &baseMode, uint32_t &customMode) {
        baseMode = 0;
        customMode = 0;

        bool found = false;

        std::map< int, std::string >::reverse_iterator iter; 
        for(iter = _modeString.rbegin(); iter != _modeString.rend(); iter++) {
            if (iter->second == flightMode) {
                baseMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
                customMode = iter->first;
                found = true;
                break;
            }
        }

        return found;
    }

    std::string pauseMode() { return "Brake"; }
    std::string loiterMode() { return "Loiter"; }
    std::string guidedMode() { return "Guided"; }

private:
    uint8_t       _firmwareType;
    uint8_t            _vehicleType;
    std::map<int , std::string> _modeString;
};

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