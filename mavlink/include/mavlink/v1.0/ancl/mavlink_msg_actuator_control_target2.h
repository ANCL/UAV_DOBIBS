#pragma once
// MESSAGE ACTUATOR_CONTROL_TARGET2 PACKING

#define MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2 194

MAVPACKED(
typedef struct __mavlink_actuator_control_target2_t {
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 float controls[4]; /*< Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1. Standard mapping for attitude controls : (index 0-3): roll, pitch, yaw, throttle*/
}) mavlink_actuator_control_target2_t;

#define MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN 24
#define MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_MIN_LEN 24
#define MAVLINK_MSG_ID_194_LEN 24
#define MAVLINK_MSG_ID_194_MIN_LEN 24

#define MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_CRC 21
#define MAVLINK_MSG_ID_194_CRC 21

#define MAVLINK_MSG_ACTUATOR_CONTROL_TARGET2_FIELD_CONTROLS_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ACTUATOR_CONTROL_TARGET2 { \
    194, \
    "ACTUATOR_CONTROL_TARGET2", \
    2, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_actuator_control_target2_t, time_usec) }, \
         { "controls", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_actuator_control_target2_t, controls) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ACTUATOR_CONTROL_TARGET2 { \
    "ACTUATOR_CONTROL_TARGET2", \
    2, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_actuator_control_target2_t, time_usec) }, \
         { "controls", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_actuator_control_target2_t, controls) }, \
         } \
}
#endif

/**
 * @brief Pack a actuator_control_target2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param controls Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1. Standard mapping for attitude controls : (index 0-3): roll, pitch, yaw, throttle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_control_target2_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, const float *controls)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float_array(buf, 8, controls, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN);
#else
    mavlink_actuator_control_target2_t packet;
    packet.time_usec = time_usec;
    mav_array_memcpy(packet.controls, controls, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_CRC);
}

/**
 * @brief Pack a actuator_control_target2 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param controls Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1. Standard mapping for attitude controls : (index 0-3): roll, pitch, yaw, throttle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_control_target2_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,const float *controls)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float_array(buf, 8, controls, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN);
#else
    mavlink_actuator_control_target2_t packet;
    packet.time_usec = time_usec;
    mav_array_memcpy(packet.controls, controls, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_CRC);
}

/**
 * @brief Encode a actuator_control_target2 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param actuator_control_target2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_actuator_control_target2_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_actuator_control_target2_t* actuator_control_target2)
{
    return mavlink_msg_actuator_control_target2_pack(system_id, component_id, msg, actuator_control_target2->time_usec, actuator_control_target2->controls);
}

/**
 * @brief Encode a actuator_control_target2 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param actuator_control_target2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_actuator_control_target2_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_actuator_control_target2_t* actuator_control_target2)
{
    return mavlink_msg_actuator_control_target2_pack_chan(system_id, component_id, chan, msg, actuator_control_target2->time_usec, actuator_control_target2->controls);
}

/**
 * @brief Send a actuator_control_target2 message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param controls Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1. Standard mapping for attitude controls : (index 0-3): roll, pitch, yaw, throttle
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_actuator_control_target2_send(mavlink_channel_t chan, uint64_t time_usec, const float *controls)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float_array(buf, 8, controls, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2, buf, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_CRC);
#else
    mavlink_actuator_control_target2_t packet;
    packet.time_usec = time_usec;
    mav_array_memcpy(packet.controls, controls, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2, (const char *)&packet, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_CRC);
#endif
}

/**
 * @brief Send a actuator_control_target2 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_actuator_control_target2_send_struct(mavlink_channel_t chan, const mavlink_actuator_control_target2_t* actuator_control_target2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_actuator_control_target2_send(chan, actuator_control_target2->time_usec, actuator_control_target2->controls);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2, (const char *)actuator_control_target2, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_CRC);
#endif
}

#if MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_actuator_control_target2_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, const float *controls)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float_array(buf, 8, controls, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2, buf, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_CRC);
#else
    mavlink_actuator_control_target2_t *packet = (mavlink_actuator_control_target2_t *)msgbuf;
    packet->time_usec = time_usec;
    mav_array_memcpy(packet->controls, controls, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2, (const char *)packet, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_CRC);
#endif
}
#endif

#endif

// MESSAGE ACTUATOR_CONTROL_TARGET2 UNPACKING


/**
 * @brief Get field time_usec from actuator_control_target2 message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_actuator_control_target2_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field controls from actuator_control_target2 message
 *
 * @return Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1. Standard mapping for attitude controls : (index 0-3): roll, pitch, yaw, throttle
 */
static inline uint16_t mavlink_msg_actuator_control_target2_get_controls(const mavlink_message_t* msg, float *controls)
{
    return _MAV_RETURN_float_array(msg, controls, 4,  8);
}

/**
 * @brief Decode a actuator_control_target2 message into a struct
 *
 * @param msg The message to decode
 * @param actuator_control_target2 C-struct to decode the message contents into
 */
static inline void mavlink_msg_actuator_control_target2_decode(const mavlink_message_t* msg, mavlink_actuator_control_target2_t* actuator_control_target2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    actuator_control_target2->time_usec = mavlink_msg_actuator_control_target2_get_time_usec(msg);
    mavlink_msg_actuator_control_target2_get_controls(msg, actuator_control_target2->controls);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN? msg->len : MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN;
        memset(actuator_control_target2, 0, MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET2_LEN);
    memcpy(actuator_control_target2, _MAV_PAYLOAD(msg), len);
#endif
}
