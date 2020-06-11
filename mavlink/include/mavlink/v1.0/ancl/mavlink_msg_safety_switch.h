#pragma once
// MESSAGE SAFETY_SWITCH PACKING

#define MAVLINK_MSG_ID_SAFETY_SWITCH 181

MAVPACKED(
typedef struct __mavlink_safety_switch_t {
 uint8_t status; /*< X|X|X|X|X|X|on/off|present*/
}) mavlink_safety_switch_t;

#define MAVLINK_MSG_ID_SAFETY_SWITCH_LEN 1
#define MAVLINK_MSG_ID_SAFETY_SWITCH_MIN_LEN 1
#define MAVLINK_MSG_ID_181_LEN 1
#define MAVLINK_MSG_ID_181_MIN_LEN 1

#define MAVLINK_MSG_ID_SAFETY_SWITCH_CRC 88
#define MAVLINK_MSG_ID_181_CRC 88



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SAFETY_SWITCH { \
    181, \
    "SAFETY_SWITCH", \
    1, \
    {  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_safety_switch_t, status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SAFETY_SWITCH { \
    "SAFETY_SWITCH", \
    1, \
    {  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_safety_switch_t, status) }, \
         } \
}
#endif

/**
 * @brief Pack a safety_switch message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status X|X|X|X|X|X|on/off|present
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_safety_switch_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SAFETY_SWITCH_LEN];
    _mav_put_uint8_t(buf, 0, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SAFETY_SWITCH_LEN);
#else
    mavlink_safety_switch_t packet;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SAFETY_SWITCH_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SAFETY_SWITCH;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SAFETY_SWITCH_MIN_LEN, MAVLINK_MSG_ID_SAFETY_SWITCH_LEN, MAVLINK_MSG_ID_SAFETY_SWITCH_CRC);
}

/**
 * @brief Pack a safety_switch message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status X|X|X|X|X|X|on/off|present
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_safety_switch_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SAFETY_SWITCH_LEN];
    _mav_put_uint8_t(buf, 0, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SAFETY_SWITCH_LEN);
#else
    mavlink_safety_switch_t packet;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SAFETY_SWITCH_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SAFETY_SWITCH;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SAFETY_SWITCH_MIN_LEN, MAVLINK_MSG_ID_SAFETY_SWITCH_LEN, MAVLINK_MSG_ID_SAFETY_SWITCH_CRC);
}

/**
 * @brief Encode a safety_switch struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param safety_switch C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_safety_switch_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_safety_switch_t* safety_switch)
{
    return mavlink_msg_safety_switch_pack(system_id, component_id, msg, safety_switch->status);
}

/**
 * @brief Encode a safety_switch struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param safety_switch C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_safety_switch_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_safety_switch_t* safety_switch)
{
    return mavlink_msg_safety_switch_pack_chan(system_id, component_id, chan, msg, safety_switch->status);
}

/**
 * @brief Send a safety_switch message
 * @param chan MAVLink channel to send the message
 *
 * @param status X|X|X|X|X|X|on/off|present
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_safety_switch_send(mavlink_channel_t chan, uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SAFETY_SWITCH_LEN];
    _mav_put_uint8_t(buf, 0, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_SWITCH, buf, MAVLINK_MSG_ID_SAFETY_SWITCH_MIN_LEN, MAVLINK_MSG_ID_SAFETY_SWITCH_LEN, MAVLINK_MSG_ID_SAFETY_SWITCH_CRC);
#else
    mavlink_safety_switch_t packet;
    packet.status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_SWITCH, (const char *)&packet, MAVLINK_MSG_ID_SAFETY_SWITCH_MIN_LEN, MAVLINK_MSG_ID_SAFETY_SWITCH_LEN, MAVLINK_MSG_ID_SAFETY_SWITCH_CRC);
#endif
}

/**
 * @brief Send a safety_switch message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_safety_switch_send_struct(mavlink_channel_t chan, const mavlink_safety_switch_t* safety_switch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_safety_switch_send(chan, safety_switch->status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_SWITCH, (const char *)safety_switch, MAVLINK_MSG_ID_SAFETY_SWITCH_MIN_LEN, MAVLINK_MSG_ID_SAFETY_SWITCH_LEN, MAVLINK_MSG_ID_SAFETY_SWITCH_CRC);
#endif
}

#if MAVLINK_MSG_ID_SAFETY_SWITCH_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_safety_switch_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_SWITCH, buf, MAVLINK_MSG_ID_SAFETY_SWITCH_MIN_LEN, MAVLINK_MSG_ID_SAFETY_SWITCH_LEN, MAVLINK_MSG_ID_SAFETY_SWITCH_CRC);
#else
    mavlink_safety_switch_t *packet = (mavlink_safety_switch_t *)msgbuf;
    packet->status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_SWITCH, (const char *)packet, MAVLINK_MSG_ID_SAFETY_SWITCH_MIN_LEN, MAVLINK_MSG_ID_SAFETY_SWITCH_LEN, MAVLINK_MSG_ID_SAFETY_SWITCH_CRC);
#endif
}
#endif

#endif

// MESSAGE SAFETY_SWITCH UNPACKING


/**
 * @brief Get field status from safety_switch message
 *
 * @return X|X|X|X|X|X|on/off|present
 */
static inline uint8_t mavlink_msg_safety_switch_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a safety_switch message into a struct
 *
 * @param msg The message to decode
 * @param safety_switch C-struct to decode the message contents into
 */
static inline void mavlink_msg_safety_switch_decode(const mavlink_message_t* msg, mavlink_safety_switch_t* safety_switch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    safety_switch->status = mavlink_msg_safety_switch_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SAFETY_SWITCH_LEN? msg->len : MAVLINK_MSG_ID_SAFETY_SWITCH_LEN;
        memset(safety_switch, 0, MAVLINK_MSG_ID_SAFETY_SWITCH_LEN);
    memcpy(safety_switch, _MAV_PAYLOAD(msg), len);
#endif
}
