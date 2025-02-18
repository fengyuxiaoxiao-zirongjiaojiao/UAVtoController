#pragma once
// MESSAGE AIRLINK_EYE_TURN_INIT PACKING

#define MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT 52005


typedef struct __mavlink_airlink_eye_turn_init_t {
 uint8_t resp_type; /*<  Turn init type*/
} mavlink_airlink_eye_turn_init_t;

#define MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN 1
#define MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_MIN_LEN 1
#define MAVLINK_MSG_ID_52005_LEN 1
#define MAVLINK_MSG_ID_52005_MIN_LEN 1

#define MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_CRC 145
#define MAVLINK_MSG_ID_52005_CRC 145



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AIRLINK_EYE_TURN_INIT { \
    52005, \
    "AIRLINK_EYE_TURN_INIT", \
    1, \
    {  { "resp_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_airlink_eye_turn_init_t, resp_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AIRLINK_EYE_TURN_INIT { \
    "AIRLINK_EYE_TURN_INIT", \
    1, \
    {  { "resp_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_airlink_eye_turn_init_t, resp_type) }, \
         } \
}
#endif

/**
 * @brief Pack a airlink_eye_turn_init message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param resp_type  Turn init type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airlink_eye_turn_init_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t resp_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN];
    _mav_put_uint8_t(buf, 0, resp_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN);
#else
    mavlink_airlink_eye_turn_init_t packet;
    packet.resp_type = resp_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_CRC);
}

/**
 * @brief Pack a airlink_eye_turn_init message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resp_type  Turn init type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airlink_eye_turn_init_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t resp_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN];
    _mav_put_uint8_t(buf, 0, resp_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN);
#else
    mavlink_airlink_eye_turn_init_t packet;
    packet.resp_type = resp_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_CRC);
}

/**
 * @brief Encode a airlink_eye_turn_init struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param airlink_eye_turn_init C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airlink_eye_turn_init_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_airlink_eye_turn_init_t* airlink_eye_turn_init)
{
    return mavlink_msg_airlink_eye_turn_init_pack(system_id, component_id, msg, airlink_eye_turn_init->resp_type);
}

/**
 * @brief Encode a airlink_eye_turn_init struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param airlink_eye_turn_init C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airlink_eye_turn_init_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_airlink_eye_turn_init_t* airlink_eye_turn_init)
{
    return mavlink_msg_airlink_eye_turn_init_pack_chan(system_id, component_id, chan, msg, airlink_eye_turn_init->resp_type);
}

/**
 * @brief Send a airlink_eye_turn_init message
 * @param chan MAVLink channel to send the message
 *
 * @param resp_type  Turn init type
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_airlink_eye_turn_init_send(mavlink_channel_t chan, uint8_t resp_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN];
    _mav_put_uint8_t(buf, 0, resp_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT, buf, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_CRC);
#else
    mavlink_airlink_eye_turn_init_t packet;
    packet.resp_type = resp_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT, (const char *)&packet, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_CRC);
#endif
}

/**
 * @brief Send a airlink_eye_turn_init message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_airlink_eye_turn_init_send_struct(mavlink_channel_t chan, const mavlink_airlink_eye_turn_init_t* airlink_eye_turn_init)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_airlink_eye_turn_init_send(chan, airlink_eye_turn_init->resp_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT, (const char *)airlink_eye_turn_init, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_CRC);
#endif
}

#if MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_airlink_eye_turn_init_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t resp_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, resp_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT, buf, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_CRC);
#else
    mavlink_airlink_eye_turn_init_t *packet = (mavlink_airlink_eye_turn_init_t *)msgbuf;
    packet->resp_type = resp_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT, (const char *)packet, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_CRC);
#endif
}
#endif

#endif

// MESSAGE AIRLINK_EYE_TURN_INIT UNPACKING


/**
 * @brief Get field resp_type from airlink_eye_turn_init message
 *
 * @return  Turn init type
 */
static inline uint8_t mavlink_msg_airlink_eye_turn_init_get_resp_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a airlink_eye_turn_init message into a struct
 *
 * @param msg The message to decode
 * @param airlink_eye_turn_init C-struct to decode the message contents into
 */
static inline void mavlink_msg_airlink_eye_turn_init_decode(const mavlink_message_t* msg, mavlink_airlink_eye_turn_init_t* airlink_eye_turn_init)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    airlink_eye_turn_init->resp_type = mavlink_msg_airlink_eye_turn_init_get_resp_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN? msg->len : MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN;
        memset(airlink_eye_turn_init, 0, MAVLINK_MSG_ID_AIRLINK_EYE_TURN_INIT_LEN);
    memcpy(airlink_eye_turn_init, _MAV_PAYLOAD(msg), len);
#endif
}
