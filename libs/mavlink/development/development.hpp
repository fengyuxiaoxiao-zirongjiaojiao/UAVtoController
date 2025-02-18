/** @file
 *	@brief MAVLink comm protocol generated from development.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace development {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (through @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 236> MESSAGE_ENTRIES {{ {0, 50, 9, 9, 0, 0, 0}, {1, 124, 31, 43, 0, 0, 0}, {2, 137, 12, 12, 0, 0, 0}, {4, 237, 14, 14, 3, 12, 13}, {5, 217, 28, 28, 1, 0, 0}, {6, 104, 3, 3, 0, 0, 0}, {7, 119, 32, 32, 0, 0, 0}, {8, 117, 36, 36, 0, 0, 0}, {11, 89, 6, 6, 1, 4, 0}, {19, 137, 24, 24, 3, 4, 5}, {20, 214, 20, 20, 3, 2, 3}, {21, 159, 2, 2, 3, 0, 1}, {22, 220, 25, 25, 0, 0, 0}, {23, 168, 23, 23, 3, 4, 5}, {24, 24, 30, 52, 0, 0, 0}, {25, 23, 101, 101, 0, 0, 0}, {26, 170, 22, 24, 0, 0, 0}, {27, 144, 26, 29, 0, 0, 0}, {28, 67, 16, 16, 0, 0, 0}, {29, 115, 14, 16, 0, 0, 0}, {30, 39, 28, 28, 0, 0, 0}, {31, 246, 32, 48, 0, 0, 0}, {32, 185, 28, 28, 0, 0, 0}, {33, 104, 28, 28, 0, 0, 0}, {34, 237, 22, 22, 0, 0, 0}, {35, 244, 22, 22, 0, 0, 0}, {36, 222, 21, 37, 0, 0, 0}, {37, 212, 6, 7, 3, 4, 5}, {38, 9, 6, 7, 3, 4, 5}, {39, 254, 37, 38, 3, 32, 33}, {40, 230, 4, 5, 3, 2, 3}, {41, 28, 4, 4, 3, 2, 3}, {42, 28, 2, 18, 0, 0, 0}, {43, 132, 2, 3, 3, 0, 1}, {44, 221, 4, 9, 3, 2, 3}, {45, 232, 2, 3, 3, 0, 1}, {46, 11, 2, 2, 0, 0, 0}, {47, 153, 3, 8, 3, 0, 1}, {48, 41, 13, 21, 1, 12, 0}, {49, 39, 12, 20, 0, 0, 0}, {50, 78, 37, 37, 3, 18, 19}, {51, 196, 4, 5, 3, 2, 3}, {54, 15, 27, 27, 3, 24, 25}, {55, 3, 25, 25, 0, 0, 0}, {61, 167, 72, 72, 0, 0, 0}, {62, 183, 26, 26, 0, 0, 0}, {63, 119, 181, 181, 0, 0, 0}, {64, 191, 225, 225, 0, 0, 0}, {65, 118, 42, 42, 0, 0, 0}, {66, 148, 6, 6, 3, 2, 3}, {67, 21, 4, 4, 0, 0, 0}, {69, 243, 11, 30, 1, 10, 0}, {70, 124, 18, 38, 3, 16, 17}, {73, 38, 37, 38, 3, 32, 33}, {74, 20, 20, 20, 0, 0, 0}, {75, 158, 35, 35, 3, 30, 31}, {76, 152, 33, 33, 3, 30, 31}, {77, 143, 3, 10, 3, 8, 9}, {80, 14, 4, 4, 3, 2, 3}, {81, 106, 22, 22, 0, 0, 0}, {82, 49, 39, 51, 3, 36, 37}, {83, 22, 37, 37, 0, 0, 0}, {84, 143, 53, 53, 3, 50, 51}, {85, 140, 51, 51, 0, 0, 0}, {86, 5, 53, 53, 3, 50, 51}, {87, 150, 51, 51, 0, 0, 0}, {89, 231, 28, 28, 0, 0, 0}, {90, 183, 56, 56, 0, 0, 0}, {91, 63, 42, 42, 0, 0, 0}, {92, 54, 33, 33, 0, 0, 0}, {93, 47, 81, 81, 0, 0, 0}, {100, 175, 26, 34, 0, 0, 0}, {101, 102, 32, 117, 0, 0, 0}, {102, 158, 32, 117, 0, 0, 0}, {103, 208, 20, 57, 0, 0, 0}, {104, 56, 32, 116, 0, 0, 0}, {105, 93, 62, 63, 0, 0, 0}, {106, 138, 44, 44, 0, 0, 0}, {107, 108, 64, 65, 0, 0, 0}, {108, 32, 84, 92, 0, 0, 0}, {109, 185, 9, 9, 0, 0, 0}, {110, 84, 254, 254, 3, 1, 2}, {111, 34, 16, 18, 3, 16, 17}, {112, 174, 12, 12, 0, 0, 0}, {113, 124, 36, 39, 0, 0, 0}, {114, 237, 44, 44, 0, 0, 0}, {115, 4, 64, 64, 0, 0, 0}, {116, 76, 22, 24, 0, 0, 0}, {117, 128, 6, 6, 3, 4, 5}, {118, 56, 14, 14, 0, 0, 0}, {119, 116, 12, 12, 3, 10, 11}, {120, 134, 97, 97, 0, 0, 0}, {121, 237, 2, 2, 3, 0, 1}, {122, 203, 2, 2, 3, 0, 1}, {123, 250, 113, 113, 3, 0, 1}, {124, 87, 35, 57, 0, 0, 0}, {125, 203, 6, 6, 0, 0, 0}, {126, 220, 79, 81, 3, 79, 80}, {127, 25, 35, 35, 0, 0, 0}, {128, 226, 35, 35, 0, 0, 0}, {129, 46, 22, 24, 0, 0, 0}, {130, 29, 13, 13, 0, 0, 0}, {131, 223, 255, 255, 0, 0, 0}, {132, 85, 14, 39, 0, 0, 0}, {133, 6, 18, 18, 0, 0, 0}, {134, 229, 43, 43, 0, 0, 0}, {135, 203, 8, 8, 0, 0, 0}, {136, 1, 22, 22, 0, 0, 0}, {137, 195, 14, 16, 0, 0, 0}, {138, 109, 36, 120, 0, 0, 0}, {139, 168, 43, 43, 3, 41, 42}, {140, 181, 41, 41, 0, 0, 0}, {141, 47, 32, 32, 0, 0, 0}, {142, 72, 243, 243, 0, 0, 0}, {143, 131, 14, 16, 0, 0, 0}, {144, 127, 93, 93, 0, 0, 0}, {146, 103, 100, 100, 0, 0, 0}, {147, 154, 36, 54, 0, 0, 0}, {148, 178, 60, 78, 0, 0, 0}, {149, 200, 30, 60, 0, 0, 0}, {162, 189, 8, 9, 0, 0, 0}, {192, 36, 44, 54, 0, 0, 0}, {225, 208, 65, 73, 0, 0, 0}, {230, 163, 42, 42, 0, 0, 0}, {231, 105, 40, 40, 0, 0, 0}, {232, 151, 63, 65, 0, 0, 0}, {233, 35, 182, 182, 0, 0, 0}, {234, 150, 40, 40, 0, 0, 0}, {235, 179, 42, 42, 0, 0, 0}, {241, 90, 32, 32, 0, 0, 0}, {242, 104, 52, 60, 0, 0, 0}, {243, 85, 53, 61, 1, 52, 0}, {244, 95, 6, 6, 0, 0, 0}, {245, 130, 2, 2, 0, 0, 0}, {246, 184, 38, 38, 0, 0, 0}, {247, 81, 19, 19, 0, 0, 0}, {248, 8, 254, 254, 3, 3, 4}, {249, 204, 36, 36, 0, 0, 0}, {250, 49, 30, 30, 0, 0, 0}, {251, 170, 18, 18, 0, 0, 0}, {252, 44, 18, 18, 0, 0, 0}, {253, 83, 51, 54, 0, 0, 0}, {254, 46, 9, 9, 0, 0, 0}, {256, 71, 42, 42, 3, 8, 9}, {257, 131, 9, 9, 0, 0, 0}, {258, 187, 32, 232, 3, 0, 1}, {259, 92, 235, 236, 0, 0, 0}, {260, 146, 5, 13, 0, 0, 0}, {261, 179, 27, 61, 0, 0, 0}, {262, 12, 18, 22, 0, 0, 0}, {263, 133, 255, 255, 0, 0, 0}, {264, 49, 28, 32, 0, 0, 0}, {265, 26, 16, 20, 0, 0, 0}, {266, 193, 255, 255, 3, 2, 3}, {267, 35, 255, 255, 3, 2, 3}, {268, 14, 4, 4, 3, 2, 3}, {269, 109, 213, 213, 0, 0, 0}, {270, 59, 19, 19, 0, 0, 0}, {271, 22, 52, 52, 0, 0, 0}, {275, 126, 31, 31, 0, 0, 0}, {276, 18, 49, 49, 0, 0, 0}, {280, 70, 33, 33, 0, 0, 0}, {281, 48, 13, 13, 0, 0, 0}, {282, 123, 35, 35, 3, 32, 33}, {283, 74, 144, 145, 0, 0, 0}, {284, 99, 32, 32, 3, 30, 31}, {285, 137, 40, 49, 3, 38, 39}, {286, 210, 53, 57, 3, 50, 51}, {287, 1, 23, 23, 3, 20, 21}, {288, 20, 23, 23, 3, 20, 21}, {290, 251, 46, 46, 0, 0, 0}, {291, 10, 57, 57, 0, 0, 0}, {295, 234, 12, 12, 0, 0, 0}, {298, 237, 37, 37, 0, 0, 0}, {299, 19, 96, 98, 0, 0, 0}, {300, 217, 22, 22, 0, 0, 0}, {301, 243, 58, 58, 0, 0, 0}, {310, 28, 17, 17, 0, 0, 0}, {311, 95, 116, 116, 0, 0, 0}, {320, 243, 20, 20, 3, 2, 3}, {321, 88, 2, 2, 3, 0, 1}, {322, 243, 149, 149, 0, 0, 0}, {323, 78, 147, 147, 3, 0, 1}, {324, 132, 146, 146, 0, 0, 0}, {330, 23, 158, 167, 0, 0, 0}, {331, 91, 230, 233, 0, 0, 0}, {332, 236, 239, 239, 0, 0, 0}, {333, 231, 109, 109, 0, 0, 0}, {334, 72, 10, 10, 0, 0, 0}, {335, 225, 24, 24, 0, 0, 0}, {336, 245, 84, 84, 0, 0, 0}, {339, 199, 5, 5, 0, 0, 0}, {340, 99, 70, 70, 0, 0, 0}, {350, 232, 20, 252, 0, 0, 0}, {354, 210, 14, 14, 3, 12, 13}, {355, 6, 12, 12, 0, 0, 0}, {360, 11, 25, 25, 0, 0, 0}, {361, 93, 33, 33, 0, 0, 0}, {369, 151, 24, 24, 0, 0, 0}, {370, 26, 140, 140, 0, 0, 0}, {373, 117, 42, 42, 0, 0, 0}, {375, 251, 140, 140, 0, 0, 0}, {380, 232, 20, 20, 0, 0, 0}, {385, 147, 133, 133, 3, 2, 3}, {386, 132, 16, 16, 3, 4, 5}, {387, 4, 72, 72, 3, 4, 5}, {388, 8, 37, 37, 3, 32, 33}, {390, 156, 238, 238, 0, 0, 0}, {395, 0, 212, 212, 0, 0, 0}, {396, 50, 160, 160, 0, 0, 0}, {397, 182, 108, 108, 0, 0, 0}, {400, 110, 254, 254, 3, 4, 5}, {401, 183, 6, 6, 3, 4, 5}, {410, 160, 53, 53, 0, 0, 0}, {411, 106, 3, 3, 0, 0, 0}, {412, 33, 6, 6, 3, 4, 5}, {413, 77, 7, 7, 3, 4, 5}, {414, 109, 16, 16, 0, 0, 0}, {415, 161, 16, 16, 0, 0, 0}, {435, 134, 46, 46, 0, 0, 0}, {436, 193, 9, 9, 0, 0, 0}, {437, 30, 1, 1, 0, 0, 0}, {510, 245, 106, 106, 0, 0, 0}, {511, 28, 71, 71, 0, 0, 0}, {9000, 113, 137, 137, 0, 0, 0}, {9005, 117, 34, 34, 0, 0, 0}, {12900, 114, 44, 44, 3, 0, 1}, {12901, 254, 59, 59, 3, 30, 31}, {12902, 140, 53, 53, 3, 4, 5}, {12903, 249, 46, 46, 3, 0, 1}, {12904, 77, 54, 54, 3, 28, 29}, {12905, 49, 43, 43, 3, 0, 1}, {12915, 94, 249, 249, 3, 0, 1}, {12918, 139, 51, 51, 0, 0, 0}, {12919, 7, 18, 18, 3, 16, 17}, {12920, 20, 5, 5, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 0;


// ENUM DEFINITIONS


/** @brief WiFi wireless security protocols. */
enum class WIFI_NETWORK_SECURITY : uint8_t
{
    UNDEFINED=0, /* Undefined or unknown security protocol. | */
    OPEN=1, /* Open network, no security. | */
    WEP=2, /* WEP. | */
    WPA1=3, /* WPA1. | */
    WPA2=4, /* WPA2. | */
    WPA3=5, /* WPA3. | */
};

//! WIFI_NETWORK_SECURITY ENUM_END
constexpr auto WIFI_NETWORK_SECURITY_ENUM_END = 6;

/** @brief Airspeed sensor flags */
enum class AIRSPEED_SENSOR_FLAGS : uint8_t
{
    UNHEALTHY=0, /* Airspeed sensor is unhealthy | */
    USING=1, /* True if the data from this sensor is being actively used by the flight controller for guidance, navigation or control. | */
};

//! AIRSPEED_SENSOR_FLAGS ENUM_END
constexpr auto AIRSPEED_SENSOR_FLAGS_ENUM_END = 2;

/** @brief Possible transport layers to set and get parameters via mavlink during a parameter transaction. */
enum class PARAM_TRANSACTION_TRANSPORT
{
    PARAM=0, /* Transaction over param transport. | */
    PARAM_EXT=1, /* Transaction over param_ext transport. | */
};

//! PARAM_TRANSACTION_TRANSPORT ENUM_END
constexpr auto PARAM_TRANSACTION_TRANSPORT_ENUM_END = 2;

/** @brief Possible parameter transaction actions. */
enum class PARAM_TRANSACTION_ACTION
{
    START=0, /* Commit the current parameter transaction. | */
    COMMIT=1, /* Commit the current parameter transaction. | */
    CANCEL=2, /* Cancel the current parameter transaction. | */
};

//! PARAM_TRANSACTION_ACTION ENUM_END
constexpr auto PARAM_TRANSACTION_ACTION_ENUM_END = 3;

/** @brief Standard modes with a well understood meaning across flight stacks and vehicle types.
        For example, most flight stack have the concept of a "return" or "RTL" mode that takes a vehicle to safety, even though the precise mechanics of this mode may differ.
        Modes may be set using MAV_CMD_DO_SET_STANDARD_MODE.
       */
enum class MAV_STANDARD_MODE : uint8_t
{
    NON_STANDARD=0, /* Non standard mode.
          This may be used when reporting the mode if the current flight mode is not a standard mode.
         | */
    POSITION_HOLD=1, /* Position mode (manual).
          Position-controlled and stabilized manual mode.
          When sticks are released vehicles return to their level-flight orientation and hold both position and altitude against wind and external forces.
          This mode can only be set by vehicles that can hold a fixed position.
          Multicopter (MC) vehicles actively brake and hold both position and altitude against wind and external forces.
          Hybrid MC/FW ("VTOL") vehicles first transition to multicopter mode (if needed) but otherwise behave in the same way as MC vehicles.
          Fixed-wing (FW) vehicles must not support this mode.
          Other vehicle types must not support this mode (this may be revisited through the PR process).
         | */
    ORBIT=2, /* Orbit (manual).
          Position-controlled and stabilized manual mode.
          The vehicle circles around a fixed setpoint in the horizontal plane at a particular radius, altitude, and direction.
          Flight stacks may further allow manual control over the setpoint position, radius, direction, speed, and/or altitude of the circle, but this is not mandated.
          Flight stacks may support the [MAV_CMD_DO_ORBIT](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_ORBIT) for changing the orbit parameters.
          MC and FW vehicles may support this mode.
          Hybrid MC/FW ("VTOL") vehicles may support this mode in MC/FW or both modes; if the mode is not supported by the current configuration the vehicle should transition to the supported configuration.
          Other vehicle types must not support this mode (this may be revisited through the PR process).
         | */
    CRUISE=3, /* Cruise mode (manual).
          Position-controlled and stabilized manual mode.
          When sticks are released vehicles return to their level-flight orientation and hold their original track against wind and external forces.
          Fixed-wing (FW) vehicles level orientation and maintain current track and altitude against wind and external forces.
          Hybrid MC/FW ("VTOL") vehicles first transition to FW mode (if needed) but otherwise behave in the same way as MC vehicles.
          Multicopter (MC) vehicles must not support this mode.
          Other vehicle types must not support this mode (this may be revisited through the PR process).
         | */
    ALTITUDE_HOLD=4, /* Altitude hold (manual).
          Altitude-controlled and stabilized manual mode.
          When sticks are released vehicles return to their level-flight orientation and hold their altitude.
          MC vehicles continue with existing momentum and may move with wind (or other external forces).
          FW vehicles continue with current heading, but may be moved off-track by wind.
          Hybrid MC/FW ("VTOL") vehicles behave according to their current configuration/mode (FW or MC).
          Other vehicle types must not support this mode (this may be revisited through the PR process).
         | */
    RETURN_HOME=5, /* Return home mode (auto).
          Automatic mode that returns vehicle to home via a safe flight path.
          It may also automatically land the vehicle (i.e. RTL).
          The precise flight path and landing behaviour depend on vehicle configuration and type.
         | */
    SAFE_RECOVERY=6, /* Safe recovery mode (auto).
          Automatic mode that takes vehicle to a predefined safe location via a safe flight path (rally point or mission defined landing) .
          It may also automatically land the vehicle.
          The precise return location, flight path, and landing behaviour depend on vehicle configuration and type.
         | */
    MISSION=7, /* Mission mode (automatic).
          Automatic mode that executes MAVLink missions.
          Missions are executed from the current waypoint as soon as the mode is enabled.
         | */
    LAND=8, /* Land mode (auto).
          Automatic mode that lands the vehicle at the current location.
          The precise landing behaviour depends on vehicle configuration and type.
         | */
    TAKEOFF=9, /* Takeoff mode (auto).
          Automatic takeoff mode.
          The precise takeoff behaviour depends on vehicle configuration and type.
         | */
};

//! MAV_STANDARD_MODE ENUM_END
constexpr auto MAV_STANDARD_MODE_ENUM_END = 10;

/** @brief Mode properties.
       */
enum class MAV_MODE_PROPERTY : uint32_t
{
    ADVANCED=1, /* If set, this mode is an advanced mode.
          For example a rate-controlled manual mode might be advanced, whereas a position-controlled manual mode is not.
          A GCS can optionally use this flag to configure the UI for its intended users.
         | */
    NOT_USER_SELECTABLE=2, /* If set, this mode should not be added to the list of selectable modes.
          The mode might still be selected by the FC directly (for example as part of a failsafe).
         | */
};

//! MAV_MODE_PROPERTY ENUM_END
constexpr auto MAV_MODE_PROPERTY_ENUM_END = 3;

/** @brief Battery status flags for fault, health and state indication. */
enum class MAV_BATTERY_STATUS_FLAGS : uint32_t
{
    NOT_READY_TO_USE=1, /* 
          The battery is not ready to use (fly).
          Set if the battery has faults or other conditions that make it unsafe to fly with.
          Note: It will be the logical OR of other status bits (chosen by the manufacturer/integrator).
         | */
    CHARGING=2, /* 
          Battery is charging.
         | */
    CELL_BALANCING=4, /* 
          Battery is cell balancing (during charging).
          Not ready to use (MAV_BATTERY_STATUS_FLAGS_NOT_READY_TO_USE may be set).
         | */
    FAULT_CELL_IMBALANCE=8, /* 
          Battery cells are not balanced.
          Not ready to use.
         | */
    AUTO_DISCHARGING=16, /* 
          Battery is auto discharging (towards storage level).
          Not ready to use (MAV_BATTERY_STATUS_FLAGS_NOT_READY_TO_USE would be set).
         | */
    REQUIRES_SERVICE=32, /* 
          Battery requires service (not safe to fly).
          This is set at vendor discretion.
          It is likely to be set for most faults, and may also be set according to a maintenance schedule (such as age, or number of recharge cycles, etc.).
         | */
    BAD_BATTERY=64, /* 
          Battery is faulty and cannot be repaired (not safe to fly).
          This is set at vendor discretion.
          The battery should be disposed of safely.
         | */
    PROTECTIONS_ENABLED=128, /* 
          Automatic battery protection monitoring is enabled.
          When enabled, the system will monitor for certain kinds of faults, such as cells being over-voltage.
          If a fault is triggered then and protections are enabled then a safety fault (MAV_BATTERY_STATUS_FLAGS_FAULT_PROTECTION_SYSTEM) will be set and power from the battery will be stopped.
          Note that battery protection monitoring should only be enabled when the vehicle is landed. Once the vehicle is armed, or starts moving, the protections should be disabled to prevent false positives from disabling the output.
         | */
    FAULT_PROTECTION_SYSTEM=256, /* 
          The battery fault protection system had detected a fault and cut all power from the battery.
          This will only trigger if MAV_BATTERY_STATUS_FLAGS_PROTECTIONS_ENABLED is set.
          Other faults like MAV_BATTERY_STATUS_FLAGS_FAULT_OVER_VOLT may also be set, indicating the cause of the protection fault.
         | */
    FAULT_OVER_VOLT=512, /* One or more cells are above their maximum voltage rating. | */
    FAULT_UNDER_VOLT=1024, /* 
          One or more cells are below their minimum voltage rating.
          A battery that had deep-discharged might be irrepairably damaged, and set both MAV_BATTERY_STATUS_FLAGS_FAULT_UNDER_VOLT and MAV_BATTERY_STATUS_FLAGS_BAD_BATTERY.
         | */
    FAULT_OVER_TEMPERATURE=2048, /* Over-temperature fault. | */
    FAULT_UNDER_TEMPERATURE=4096, /* Under-temperature fault. | */
    FAULT_OVER_CURRENT=8192, /* Over-current fault. | */
    FAULT_SHORT_CIRCUIT=16384, /* 
          Short circuit event detected.
          The battery may or may not be safe to use (check other flags).
         | */
    FAULT_INCOMPATIBLE_VOLTAGE=32768, /* Voltage not compatible with power rail voltage (batteries on same power rail should have similar voltage). | */
    FAULT_INCOMPATIBLE_FIRMWARE=65536, /* Battery firmware is not compatible with current autopilot firmware. | */
    FAULT_INCOMPATIBLE_CELLS_CONFIGURATION=131072, /* Battery is not compatible due to cell configuration (e.g. 5s1p when vehicle requires 6s). | */
    CAPACITY_RELATIVE_TO_FULL=262144, /* 
          Battery capacity_consumed and capacity_remaining values are relative to a full battery (they sum to the total capacity of the battery).
          This flag would be set for a smart battery that can accurately determine its remaining charge across vehicle reboots and discharge/recharge cycles.
          If unset the capacity_consumed indicates the consumption since vehicle power-on, as measured using a power monitor. The capacity_remaining, if provided, indicates the estimated remaining capacity on the assumption that the battery was full on vehicle boot.
          If unset a GCS is recommended to advise that users fully charge the battery on power on.
         | */
    EXTENDED=4294967295, /* Reserved (not used). If set, this will indicate that an additional status field exists for higher status values. | */
};

//! MAV_BATTERY_STATUS_FLAGS ENUM_END
constexpr auto MAV_BATTERY_STATUS_FLAGS_ENUM_END = 4294967296;

/** @brief These flags indicate the sensor reporting capabilities for TARGET_ABSOLUTE. */
enum class TARGET_ABSOLUTE_SENSOR_CAPABILITY_FLAGS : uint8_t
{
    POSITION=1, /*  | */
    VELOCITY=2, /*  | */
    ACCELERATION=4, /*  | */
    ATTITUDE=8, /*  | */
    RATES=16, /*  | */
};

//! TARGET_ABSOLUTE_SENSOR_CAPABILITY_FLAGS ENUM_END
constexpr auto TARGET_ABSOLUTE_SENSOR_CAPABILITY_FLAGS_ENUM_END = 17;

/** @brief The frame of a target observation from an onboard sensor. */
enum class TARGET_OBS_FRAME : uint8_t
{
    LOCAL_NED=0, /* NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth. | */
    BODY_FRD=1, /* FRD local frame aligned to the vehicle's attitude (x: Forward, y: Right, z: Down) with an origin that travels with vehicle. | */
    LOCAL_OFFSET_NED=2, /* NED local tangent frame (x: North, y: East, z: Down) with an origin that travels with vehicle. | */
    OTHER=3, /* Other sensor frame for target observations neither in local NED nor in body FRD. | */
};

//! TARGET_OBS_FRAME ENUM_END
constexpr auto TARGET_OBS_FRAME_ENUM_END = 4;


} // namespace development
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_param_ack_transaction.hpp"
#include "./mavlink_msg_airspeed.hpp"
#include "./mavlink_msg_wifi_network_info.hpp"
#include "./mavlink_msg_set_velocity_limits.hpp"
#include "./mavlink_msg_velocity_limits.hpp"
#include "./mavlink_msg_figure_eight_execution_status.hpp"
#include "./mavlink_msg_battery_status_v2.hpp"
#include "./mavlink_msg_component_information_basic.hpp"
#include "./mavlink_msg_group_start.hpp"
#include "./mavlink_msg_group_end.hpp"
#include "./mavlink_msg_available_modes.hpp"
#include "./mavlink_msg_current_mode.hpp"
#include "./mavlink_msg_available_modes_monitor.hpp"
#include "./mavlink_msg_target_absolute.hpp"
#include "./mavlink_msg_target_relative.hpp"

// base include
#include "../common/common.hpp"
