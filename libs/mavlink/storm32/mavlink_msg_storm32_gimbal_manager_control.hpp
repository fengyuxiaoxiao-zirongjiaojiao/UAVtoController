// MESSAGE STORM32_GIMBAL_MANAGER_CONTROL support class

#pragma once

namespace mavlink {
namespace storm32 {
namespace msg {

/**
 * @brief STORM32_GIMBAL_MANAGER_CONTROL message
 *
 * Message to a gimbal manager to control the gimbal attitude. Angles and rates can be set to NaN according to use case. A gimbal device is never to react to this message.
 */
struct STORM32_GIMBAL_MANAGER_CONTROL : mavlink::Message {
    static constexpr msgid_t MSG_ID = 60012;
    static constexpr size_t LENGTH = 36;
    static constexpr size_t MIN_LENGTH = 36;
    static constexpr uint8_t CRC_EXTRA = 99;
    static constexpr auto NAME = "STORM32_GIMBAL_MANAGER_CONTROL";


    uint8_t target_system; /*<  System ID */
    uint8_t target_component; /*<  Component ID */
    uint8_t gimbal_id; /*<  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals). Send command multiple times for more than one but not all gimbals. */
    uint8_t client; /*<  Client which is contacting the gimbal manager (must be set). */
    uint16_t device_flags; /*<  Gimbal device flags to be applied (UINT16_MAX to be ignored). Same flags as used in GIMBAL_DEVICE_SET_ATTITUDE. */
    uint16_t manager_flags; /*<  Gimbal manager flags to be applied (0 to be ignored). */
    std::array<float, 4> q; /*<  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation). Set first element to NaN to be ignored. The frame is determined by the GIMBAL_DEVICE_FLAGS_YAW_IN_xxx_FRAME flags. */
    float angular_velocity_x; /*< [rad/s] X component of angular velocity (positive: roll to the right). NaN to be ignored. */
    float angular_velocity_y; /*< [rad/s] Y component of angular velocity (positive: tilt up). NaN to be ignored. */
    float angular_velocity_z; /*< [rad/s] Z component of angular velocity (positive: pan to the right). NaN to be ignored. The frame is determined by the GIMBAL_DEVICE_FLAGS_YAW_IN_xxx_FRAME flags. */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  target_system: " << +target_system << std::endl;
        ss << "  target_component: " << +target_component << std::endl;
        ss << "  gimbal_id: " << +gimbal_id << std::endl;
        ss << "  client: " << +client << std::endl;
        ss << "  device_flags: " << device_flags << std::endl;
        ss << "  manager_flags: " << manager_flags << std::endl;
        ss << "  q: [" << to_string(q) << "]" << std::endl;
        ss << "  angular_velocity_x: " << angular_velocity_x << std::endl;
        ss << "  angular_velocity_y: " << angular_velocity_y << std::endl;
        ss << "  angular_velocity_z: " << angular_velocity_z << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << q;                             // offset: 0
        map << angular_velocity_x;            // offset: 16
        map << angular_velocity_y;            // offset: 20
        map << angular_velocity_z;            // offset: 24
        map << device_flags;                  // offset: 28
        map << manager_flags;                 // offset: 30
        map << target_system;                 // offset: 32
        map << target_component;              // offset: 33
        map << gimbal_id;                     // offset: 34
        map << client;                        // offset: 35
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> q;                             // offset: 0
        map >> angular_velocity_x;            // offset: 16
        map >> angular_velocity_y;            // offset: 20
        map >> angular_velocity_z;            // offset: 24
        map >> device_flags;                  // offset: 28
        map >> manager_flags;                 // offset: 30
        map >> target_system;                 // offset: 32
        map >> target_component;              // offset: 33
        map >> gimbal_id;                     // offset: 34
        map >> client;                        // offset: 35
    }
};

} // namespace msg
} // namespace storm32
} // namespace mavlink
