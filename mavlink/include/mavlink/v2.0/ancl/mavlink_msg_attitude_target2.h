#pragma once
// MESSAGE ATTITUDE_TARGET2 PACKING

#define MAVLINK_MSG_ID_ATTITUDE_TARGET2 193

MAVPACKED(
typedef struct __mavlink_attitude_target2_t {
 uint32_t time_boot_ms; /*< Timestamp in milliseconds since system boot*/
 float q[4]; /*< Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)*/
 float roll; /*< Body roll angle in radians*/
 float pitch; /*< Body pitch in radians*/
 float yaw; /*< Body yaw angle in radians*/
 float thrust; /*< Collective thrust, normalized to 0 .. 1*/
 uint8_t valid; /*< Valid*/
}) mavlink_attitude_target2_t;

#define MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN 37
#define MAVLINK_MSG_ID_ATTITUDE_TARGET2_MIN_LEN 37
#define MAVLINK_MSG_ID_193_LEN 37
#define MAVLINK_MSG_ID_193_MIN_LEN 37

#define MAVLINK_MSG_ID_ATTITUDE_TARGET2_CRC 164
#define MAVLINK_MSG_ID_193_CRC 164

#define MAVLINK_MSG_ATTITUDE_TARGET2_FIELD_Q_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ATTITUDE_TARGET2 { \
    193, \
    "ATTITUDE_TARGET2", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_attitude_target2_t, time_boot_ms) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_attitude_target2_t, q) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_target2_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude_target2_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_attitude_target2_t, yaw) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_attitude_target2_t, thrust) }, \
         { "valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_attitude_target2_t, valid) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ATTITUDE_TARGET2 { \
    "ATTITUDE_TARGET2", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_attitude_target2_t, time_boot_ms) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_attitude_target2_t, q) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_target2_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude_target2_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_attitude_target2_t, yaw) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_attitude_target2_t, thrust) }, \
         { "valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_attitude_target2_t, valid) }, \
         } \
}
#endif

/**
 * @brief Pack a attitude_target2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param q Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param roll Body roll angle in radians
 * @param pitch Body pitch in radians
 * @param yaw Body yaw angle in radians
 * @param thrust Collective thrust, normalized to 0 .. 1
 * @param valid Valid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_target2_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, const float *q, float roll, float pitch, float yaw, float thrust, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);
    _mav_put_float(buf, 32, thrust);
    _mav_put_uint8_t(buf, 36, valid);
    _mav_put_float_array(buf, 4, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN);
#else
    mavlink_attitude_target2_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.thrust = thrust;
    packet.valid = valid;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE_TARGET2;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE_TARGET2_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET2_CRC);
}

/**
 * @brief Pack a attitude_target2 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param q Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param roll Body roll angle in radians
 * @param pitch Body pitch in radians
 * @param yaw Body yaw angle in radians
 * @param thrust Collective thrust, normalized to 0 .. 1
 * @param valid Valid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_target2_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,const float *q,float roll,float pitch,float yaw,float thrust,uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);
    _mav_put_float(buf, 32, thrust);
    _mav_put_uint8_t(buf, 36, valid);
    _mav_put_float_array(buf, 4, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN);
#else
    mavlink_attitude_target2_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.thrust = thrust;
    packet.valid = valid;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE_TARGET2;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE_TARGET2_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET2_CRC);
}

/**
 * @brief Encode a attitude_target2 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude_target2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_target2_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_target2_t* attitude_target2)
{
    return mavlink_msg_attitude_target2_pack(system_id, component_id, msg, attitude_target2->time_boot_ms, attitude_target2->q, attitude_target2->roll, attitude_target2->pitch, attitude_target2->yaw, attitude_target2->thrust, attitude_target2->valid);
}

/**
 * @brief Encode a attitude_target2 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude_target2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_target2_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_attitude_target2_t* attitude_target2)
{
    return mavlink_msg_attitude_target2_pack_chan(system_id, component_id, chan, msg, attitude_target2->time_boot_ms, attitude_target2->q, attitude_target2->roll, attitude_target2->pitch, attitude_target2->yaw, attitude_target2->thrust, attitude_target2->valid);
}

/**
 * @brief Send a attitude_target2 message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param q Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param roll Body roll angle in radians
 * @param pitch Body pitch in radians
 * @param yaw Body yaw angle in radians
 * @param thrust Collective thrust, normalized to 0 .. 1
 * @param valid Valid
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_target2_send(mavlink_channel_t chan, uint32_t time_boot_ms, const float *q, float roll, float pitch, float yaw, float thrust, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);
    _mav_put_float(buf, 32, thrust);
    _mav_put_uint8_t(buf, 36, valid);
    _mav_put_float_array(buf, 4, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TARGET2, buf, MAVLINK_MSG_ID_ATTITUDE_TARGET2_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET2_CRC);
#else
    mavlink_attitude_target2_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.thrust = thrust;
    packet.valid = valid;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TARGET2, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE_TARGET2_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET2_CRC);
#endif
}

/**
 * @brief Send a attitude_target2 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_attitude_target2_send_struct(mavlink_channel_t chan, const mavlink_attitude_target2_t* attitude_target2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_attitude_target2_send(chan, attitude_target2->time_boot_ms, attitude_target2->q, attitude_target2->roll, attitude_target2->pitch, attitude_target2->yaw, attitude_target2->thrust, attitude_target2->valid);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TARGET2, (const char *)attitude_target2, MAVLINK_MSG_ID_ATTITUDE_TARGET2_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET2_CRC);
#endif
}

#if MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_attitude_target2_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, const float *q, float roll, float pitch, float yaw, float thrust, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);
    _mav_put_float(buf, 32, thrust);
    _mav_put_uint8_t(buf, 36, valid);
    _mav_put_float_array(buf, 4, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TARGET2, buf, MAVLINK_MSG_ID_ATTITUDE_TARGET2_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET2_CRC);
#else
    mavlink_attitude_target2_t *packet = (mavlink_attitude_target2_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->thrust = thrust;
    packet->valid = valid;
    mav_array_memcpy(packet->q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_TARGET2, (const char *)packet, MAVLINK_MSG_ID_ATTITUDE_TARGET2_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN, MAVLINK_MSG_ID_ATTITUDE_TARGET2_CRC);
#endif
}
#endif

#endif

// MESSAGE ATTITUDE_TARGET2 UNPACKING


/**
 * @brief Get field time_boot_ms from attitude_target2 message
 *
 * @return Timestamp in milliseconds since system boot
 */
static inline uint32_t mavlink_msg_attitude_target2_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field q from attitude_target2 message
 *
 * @return Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 */
static inline uint16_t mavlink_msg_attitude_target2_get_q(const mavlink_message_t* msg, float *q)
{
    return _MAV_RETURN_float_array(msg, q, 4,  4);
}

/**
 * @brief Get field roll from attitude_target2 message
 *
 * @return Body roll angle in radians
 */
static inline float mavlink_msg_attitude_target2_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pitch from attitude_target2 message
 *
 * @return Body pitch in radians
 */
static inline float mavlink_msg_attitude_target2_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yaw from attitude_target2 message
 *
 * @return Body yaw angle in radians
 */
static inline float mavlink_msg_attitude_target2_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field thrust from attitude_target2 message
 *
 * @return Collective thrust, normalized to 0 .. 1
 */
static inline float mavlink_msg_attitude_target2_get_thrust(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field valid from attitude_target2 message
 *
 * @return Valid
 */
static inline uint8_t mavlink_msg_attitude_target2_get_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Decode a attitude_target2 message into a struct
 *
 * @param msg The message to decode
 * @param attitude_target2 C-struct to decode the message contents into
 */
static inline void mavlink_msg_attitude_target2_decode(const mavlink_message_t* msg, mavlink_attitude_target2_t* attitude_target2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    attitude_target2->time_boot_ms = mavlink_msg_attitude_target2_get_time_boot_ms(msg);
    mavlink_msg_attitude_target2_get_q(msg, attitude_target2->q);
    attitude_target2->roll = mavlink_msg_attitude_target2_get_roll(msg);
    attitude_target2->pitch = mavlink_msg_attitude_target2_get_pitch(msg);
    attitude_target2->yaw = mavlink_msg_attitude_target2_get_yaw(msg);
    attitude_target2->thrust = mavlink_msg_attitude_target2_get_thrust(msg);
    attitude_target2->valid = mavlink_msg_attitude_target2_get_valid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN? msg->len : MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN;
        memset(attitude_target2, 0, MAVLINK_MSG_ID_ATTITUDE_TARGET2_LEN);
    memcpy(attitude_target2, _MAV_PAYLOAD(msg), len);
#endif
}
