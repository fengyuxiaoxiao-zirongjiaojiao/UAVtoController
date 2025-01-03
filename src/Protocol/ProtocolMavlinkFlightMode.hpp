/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2025-01-03 10:38:27
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2025-01-03 13:43:55
 * @FilePath: /UAVtoController/src/Protocol/ProtocolMavlinkFlightMode.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __PROTOCOL_MAVLINK_FLIGHTMODE_H_
#define __PROTOCOL_MAVLINK_FLIGHTMODE_H_

#include <iostream>
#include <map>
#include <vector>

class FlightModeInterface
{
public:
    FlightModeInterface() {}
    virtual std::vector<std::string> flightModes() = 0;
    virtual std::string flightMode(uint8_t baseMode, uint32_t customMode) = 0;
    virtual bool setFlightMode(const std::string flightMode, uint8_t &baseMode, uint32_t &customMode) = 0;

    virtual std::string loiterMode() = 0;       // 定点 悬停
    virtual std::string landMode() = 0;         // 降落
    virtual std::string rtlMode() = 0;          // 返航
    virtual std::string stabilizedMode() = 0;   // 增稳
    virtual std::string hoverMode() = 0;        // 定高
    virtual std::string autoMode() = 0;         // 自动 航线模式
    virtual std::string guidedMode() = 0;       // 引导模式  或者  offboard
    virtual std::string pauseMode() = 0;        // 暂停
};

class ApmMultiRotorFlightMode : public FlightModeInterface
{
public:
    ApmMultiRotorFlightMode();
    ~ApmMultiRotorFlightMode();

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
    
    virtual std::vector<std::string> flightModes() override;
    virtual std::string flightMode(uint8_t baseMode, uint32_t customMode) override;
    virtual bool setFlightMode(const std::string flightMode, uint8_t &baseMode, uint32_t &customMode) override;

    virtual std::string loiterMode() { return "Loiter"; }           // 定点 悬停
    virtual std::string landMode() { return "Land"; }               // 降落
    virtual std::string rtlMode() { return "RTL"; }                 // 返航
    virtual std::string stabilizedMode() { return "Stabilize"; }    // 增稳
    virtual std::string hoverMode() { return "Altitude Hold"; }     // 定高
    virtual std::string autoMode() { return "Auto";}                // 自动 航线模式
    virtual std::string guidedMode() { return "Guided"; }           // 引导模式  或者  offboard
    virtual std::string pauseMode() { return "Brake"; }             // 暂停

private:
    std::map<int , std::string> _modeString;
};


class PX4MultiRotorFlightMode : public FlightModeInterface
{
public:
    PX4MultiRotorFlightMode();
    ~PX4MultiRotorFlightMode();

    enum PX4_CUSTOM_MAIN_MODE {
        PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
        PX4_CUSTOM_MAIN_MODE_ALTCTL,
        PX4_CUSTOM_MAIN_MODE_POSCTL,
        PX4_CUSTOM_MAIN_MODE_AUTO,
        PX4_CUSTOM_MAIN_MODE_ACRO,
        PX4_CUSTOM_MAIN_MODE_OFFBOARD,
        PX4_CUSTOM_MAIN_MODE_STABILIZED,
        PX4_CUSTOM_MAIN_MODE_RATTITUDE,
        PX4_CUSTOM_MAIN_MODE_SIMPLE /* unused, but reserved for future use */
    };

    enum PX4_CUSTOM_SUB_MODE_AUTO {
        PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
        PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
        PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
        PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
        PX4_CUSTOM_SUB_MODE_AUTO_RTL,
        PX4_CUSTOM_SUB_MODE_AUTO_LAND,
        PX4_CUSTOM_SUB_MODE_AUTO_RTGS,
        PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET,
        PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND
    };

    enum PX4_CUSTOM_SUB_MODE_POSCTL {
        PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL = 0,
        PX4_CUSTOM_SUB_MODE_POSCTL_ORBIT
    };

    typedef struct {
        uint8_t         main_mode;
        uint8_t         sub_mode;
        const std::string  name;       ///< Name for flight mode
        bool            canBeSet;   ///< true: Vehicle can be set to this flight mode
        bool            fixedWing;  /// fixed wing compatible
        bool            multiRotor; /// multi rotor compatible
        bool            isSimple;   /// true:用于普通用户操作的基本模式
    } FlightModeInfo_t;
    union px4_custom_mode {
        struct {
            uint16_t reserved;
            uint8_t main_mode;
            uint8_t sub_mode;
        };
        uint32_t data;
        float data_float;
        struct {
            uint16_t reserved_hl;
            uint16_t custom_mode_hl;
        };
    };

    virtual std::vector<std::string> flightModes() override;
    virtual std::string flightMode(uint8_t baseMode, uint32_t customMode) override;
    virtual bool setFlightMode(const std::string flightMode, uint8_t &baseMode, uint32_t &customMode) override;

    virtual std::string loiterMode() { return "Hold"; }                 // 定点 悬停
    virtual std::string landMode() { return "Land"; }                   // 降落
    virtual std::string rtlMode() { return "Return"; }                  // 返航
    virtual std::string stabilizedMode() { return "Stabilized"; }       // 增稳
    virtual std::string hoverMode() { return "Altitude"; }              // 定高
    virtual std::string autoMode() { return "Mission";}                 // 自动 航线模式
    virtual std::string guidedMode() { return "Offboard"; }             // 引导模式  或者  offboard
    virtual std::string pauseMode() { return loiterMode(); }            // 暂停

private:
    std::vector<FlightModeInfo_t> _flightModeInfoList;
};

#endif //__PROTOCOL_MAVLINK_FLIGHTMODE_H_