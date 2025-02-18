// MESSAGE SATCOM_LINK_STATUS support class

#pragma once

namespace mavlink {
namespace ASLUAV {
namespace msg {

/**
 * @brief SATCOM_LINK_STATUS message
 *
 * Status of the SatCom link
 */
struct SATCOM_LINK_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 8015;
    static constexpr size_t LENGTH = 24;
    static constexpr size_t MIN_LENGTH = 24;
    static constexpr uint8_t CRC_EXTRA = 23;
    static constexpr auto NAME = "SATCOM_LINK_STATUS";


    uint64_t timestamp; /*< [us] Timestamp */
    uint64_t last_heartbeat; /*< [us] Timestamp of the last successful sbd session */
    uint16_t failed_sessions; /*<  Number of failed sessions */
    uint16_t successful_sessions; /*<  Number of successful sessions */
    uint8_t signal_quality; /*<  Signal quality */
    uint8_t ring_pending; /*<  Ring call pending */
    uint8_t tx_session_pending; /*<  Transmission session pending */
    uint8_t rx_session_pending; /*<  Receiving session pending */


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
        ss << "  timestamp: " << timestamp << std::endl;
        ss << "  last_heartbeat: " << last_heartbeat << std::endl;
        ss << "  failed_sessions: " << failed_sessions << std::endl;
        ss << "  successful_sessions: " << successful_sessions << std::endl;
        ss << "  signal_quality: " << +signal_quality << std::endl;
        ss << "  ring_pending: " << +ring_pending << std::endl;
        ss << "  tx_session_pending: " << +tx_session_pending << std::endl;
        ss << "  rx_session_pending: " << +rx_session_pending << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << timestamp;                     // offset: 0
        map << last_heartbeat;                // offset: 8
        map << failed_sessions;               // offset: 16
        map << successful_sessions;           // offset: 18
        map << signal_quality;                // offset: 20
        map << ring_pending;                  // offset: 21
        map << tx_session_pending;            // offset: 22
        map << rx_session_pending;            // offset: 23
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> timestamp;                     // offset: 0
        map >> last_heartbeat;                // offset: 8
        map >> failed_sessions;               // offset: 16
        map >> successful_sessions;           // offset: 18
        map >> signal_quality;                // offset: 20
        map >> ring_pending;                  // offset: 21
        map >> tx_session_pending;            // offset: 22
        map >> rx_session_pending;            // offset: 23
    }
};

} // namespace msg
} // namespace ASLUAV
} // namespace mavlink
