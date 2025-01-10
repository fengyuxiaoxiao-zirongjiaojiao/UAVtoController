#include "ProtocolMavlinkFlightMode.hpp"
#include "mavlink.h"
#include "Logger.hpp"

ApmMultiRotorFlightMode::ApmMultiRotorFlightMode()
{
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

ApmMultiRotorFlightMode::~ApmMultiRotorFlightMode()
{

}

std::vector<std::string> ApmMultiRotorFlightMode::flightModes()
{
    std::vector<std::string> modes;
    std::map<int , std::string>::iterator it = _modeString.begin();
    for (it = _modeString.begin(); it != _modeString.end(); it++) {
        modes.push_back(it->second);
    }
    return modes;
}

std::string ApmMultiRotorFlightMode::flightMode(uint8_t baseMode, uint32_t customMode)
{
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

bool ApmMultiRotorFlightMode::setFlightMode(const std::string flightMode, uint8_t &baseMode, uint32_t &customMode)
{
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

PX4MultiRotorFlightMode::PX4MultiRotorFlightMode()
{
                                                    //main_mode                         sub_mode                             name                       canBeSet  FW      MC  isSimple
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_MANUAL,      0,                                     "Manual",                  true,   true,   true,  false });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_STABILIZED,  0,                                     "Stabilized",              true,   true,   true,  false });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_ACRO,        0,                                     "Acro",                    true,   true,   true,  false });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_RATTITUDE,   0,                                     "Rattitude",               true,   true,   true,  true });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_ALTCTL,      0,                                     "Altitude",                true,   true,   true,  true });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_OFFBOARD,    0,                                     "Offboard",                true,   false,  true,  true });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_SIMPLE,      0,                                     "Simple",                  false,  false,  true,  true });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_POSCTL,      PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL,     "Position",                true,   true,   true,  false });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_POSCTL,      PX4_CUSTOM_SUB_MODE_POSCTL_ORBIT,      "Orbit",                   false,  false,  false, false });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_LOITER,       "Hold",                    true,   true,   true,  true });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_MISSION,      "Mission",                 true,   true,   true,  true });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_RTL,          "Return",                  true,   true,   true,  true });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET,"Follow Me",               true,   false,  true,  true });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_LAND,         "Land",                    false,  true,   true,  false });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND,     "Precision Land",          false,  false,  true,  false });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_READY,        "Ready",                   false,  true,   true,  false });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_RTGS,         "Return to Groundstation", false,  true,   true,  false });
    _flightModeInfoList.push_back(FlightModeInfo_t{ PX4_CUSTOM_MAIN_MODE_AUTO,        PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,      "Takeoff",                 false,  true,   true,  false });
}

PX4MultiRotorFlightMode::~PX4MultiRotorFlightMode()
{

}

std::vector<std::string> PX4MultiRotorFlightMode::flightModes()
{
    std::vector<std::string> flightModes;

    for (const FlightModeInfo_t& info : _flightModeInfoList) {
        if (info.canBeSet) {
            flightModes.push_back(info.name);
        }
    }

    return flightModes;
}

std::string PX4MultiRotorFlightMode::flightMode(uint8_t baseMode, uint32_t customMode)
{
    std::string flightMode = "Unknown";

    if (baseMode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
        union px4_custom_mode px4_mode;
        px4_mode.data = customMode;

        bool found = false;
        for (const FlightModeInfo_t& info : _flightModeInfoList) {
            if (info.main_mode == px4_mode.main_mode && info.sub_mode == px4_mode.sub_mode) {
                flightMode = info.name;
                found = true;
                break;
            }
        }

        if (!found) {
            Logger::getInstance()->log(LOGLEVEL_WARNING, "Unknown flight mode:" + std::to_string(baseMode) + " " + std::to_string(customMode));
            return std::string("Unknown ") + std::to_string(baseMode) + std::to_string(customMode);
        }
    }

    return flightMode;
}

bool PX4MultiRotorFlightMode::setFlightMode(const std::string flightMode, uint8_t &baseMode, uint32_t &customMode)
{
    baseMode = 0;
    customMode = 0;

    bool found = false;
    for (const FlightModeInfo_t& info : _flightModeInfoList) {
        if (flightMode.compare(info.name) == 0) {
            union px4_custom_mode px4_mode;
            px4_mode.data = 0;
            px4_mode.main_mode = info.main_mode;
            px4_mode.sub_mode = info.sub_mode;

            baseMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
            customMode = px4_mode.data;

            found = true;
            break;
        }
    }

    if (!found) {
        std::cout << "Unknown flight Mode" << flightMode;
    }

    return found;
}