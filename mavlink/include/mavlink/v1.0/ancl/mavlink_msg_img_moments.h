#pragma once
// MESSAGE IMG_MOMENTS PACKING

#define MAVLINK_MSG_ID_IMG_MOMENTS 190

MAVPACKED(
typedef struct __mavlink_img_moments_t {
 uint32_t usec; /*< Timestamp (microseconds)*/
 float s[4]; /*< See Thesis for definitions*/
 uint8_t valid; /*< Message Vadilidity*/
}) mavlink_img_moments_t;

#define MAVLINK_MSG_ID_IMG_MOMENTS_LEN 21
#define MAVLINK_MSG_ID_IMG_MOMENTS_MIN_LEN 21
#define MAVLINK_MSG_ID_190_LEN 21
#define MAVLINK_MSG_ID_190_MIN_LEN 21

#define MAVLINK_MSG_ID_IMG_MOMENTS_CRC 76
#define MAVLINK_MSG_ID_190_CRC 76

#define MAVLINK_MSG_IMG_MOMENTS_FIELD_S_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_IMG_MOMENTS { \
    190, \
    "IMG_MOMENTS", \
    3, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_img_moments_t, usec) }, \
         { "s", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_img_moments_t, s) }, \
         { "valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_img_moments_t, valid) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_IMG_MOMENTS { \
    "IMG_MOMENTS", \
    3, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_img_moments_t, usec) }, \
         { "s", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_img_moments_t, s) }, \
         { "valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_img_moments_t, valid) }, \
         } \
}
#endif

/**
 * @brief Pack a img_moments message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds)
 * @param s See Thesis for definitions
 * @param valid Message Vadilidity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_img_moments_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t usec, const float *s, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMG_MOMENTS_LEN];
    _mav_put_uint32_t(buf, 0, usec);
    _mav_put_uint8_t(buf, 20, valid);
    _mav_put_float_array(buf, 4, s, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMG_MOMENTS_LEN);
#else
    mavlink_img_moments_t packet;
    packet.usec = usec;
    packet.valid = valid;
    mav_array_memcpy(packet.s, s, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMG_MOMENTS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMG_MOMENTS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMG_MOMENTS_MIN_LEN, MAVLINK_MSG_ID_IMG_MOMENTS_LEN, MAVLINK_MSG_ID_IMG_MOMENTS_CRC);
}

/**
 * @brief Pack a img_moments message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds)
 * @param s See Thesis for definitions
 * @param valid Message Vadilidity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_img_moments_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t usec,const float *s,uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMG_MOMENTS_LEN];
    _mav_put_uint32_t(buf, 0, usec);
    _mav_put_uint8_t(buf, 20, valid);
    _mav_put_float_array(buf, 4, s, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMG_MOMENTS_LEN);
#else
    mavlink_img_moments_t packet;
    packet.usec = usec;
    packet.valid = valid;
    mav_array_memcpy(packet.s, s, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMG_MOMENTS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMG_MOMENTS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMG_MOMENTS_MIN_LEN, MAVLINK_MSG_ID_IMG_MOMENTS_LEN, MAVLINK_MSG_ID_IMG_MOMENTS_CRC);
}

/**
 * @brief Encode a img_moments struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param img_moments C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_img_moments_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_img_moments_t* img_moments)
{
    return mavlink_msg_img_moments_pack(system_id, component_id, msg, img_moments->usec, img_moments->s, img_moments->valid);
}

/**
 * @brief Encode a img_moments struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param img_moments C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_img_moments_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_img_moments_t* img_moments)
{
    return mavlink_msg_img_moments_pack_chan(system_id, component_id, chan, msg, img_moments->usec, img_moments->s, img_moments->valid);
}

/**
 * @brief Send a img_moments message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds)
 * @param s See Thesis for definitions
 * @param valid Message Vadilidity
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_img_moments_send(mavlink_channel_t chan, uint32_t usec, const float *s, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMG_MOMENTS_LEN];
    _mav_put_uint32_t(buf, 0, usec);
    _mav_put_uint8_t(buf, 20, valid);
    _mav_put_float_array(buf, 4, s, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMG_MOMENTS, buf, MAVLINK_MSG_ID_IMG_MOMENTS_MIN_LEN, MAVLINK_MSG_ID_IMG_MOMENTS_LEN, MAVLINK_MSG_ID_IMG_MOMENTS_CRC);
#else
    mavlink_img_moments_t packet;
    packet.usec = usec;
    packet.valid = valid;
    mav_array_memcpy(packet.s, s, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMG_MOMENTS, (const char *)&packet, MAVLINK_MSG_ID_IMG_MOMENTS_MIN_LEN, MAVLINK_MSG_ID_IMG_MOMENTS_LEN, MAVLINK_MSG_ID_IMG_MOMENTS_CRC);
#endif
}

/**
 * @brief Send a img_moments message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_img_moments_send_struct(mavlink_channel_t chan, const mavlink_img_moments_t* img_moments)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_img_moments_send(chan, img_moments->usec, img_moments->s, img_moments->valid);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMG_MOMENTS, (const char *)img_moments, MAVLINK_MSG_ID_IMG_MOMENTS_MIN_LEN, MAVLINK_MSG_ID_IMG_MOMENTS_LEN, MAVLINK_MSG_ID_IMG_MOMENTS_CRC);
#endif
}

#if MAVLINK_MSG_ID_IMG_MOMENTS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_img_moments_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t usec, const float *s, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, usec);
    _mav_put_uint8_t(buf, 20, valid);
    _mav_put_float_array(buf, 4, s, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMG_MOMENTS, buf, MAVLINK_MSG_ID_IMG_MOMENTS_MIN_LEN, MAVLINK_MSG_ID_IMG_MOMENTS_LEN, MAVLINK_MSG_ID_IMG_MOMENTS_CRC);
#else
    mavlink_img_moments_t *packet = (mavlink_img_moments_t *)msgbuf;
    packet->usec = usec;
    packet->valid = valid;
    mav_array_memcpy(packet->s, s, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMG_MOMENTS, (const char *)packet, MAVLINK_MSG_ID_IMG_MOMENTS_MIN_LEN, MAVLINK_MSG_ID_IMG_MOMENTS_LEN, MAVLINK_MSG_ID_IMG_MOMENTS_CRC);
#endif
}
#endif

#endif

// MESSAGE IMG_MOMENTS UNPACKING


/**
 * @brief Get field usec from img_moments message
 *
 * @return Timestamp (microseconds)
 */
static inline uint32_t mavlink_msg_img_moments_get_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field s from img_moments message
 *
 * @return See Thesis for definitions
 */
static inline uint16_t mavlink_msg_img_moments_get_s(const mavlink_message_t* msg, float *s)
{
    return _MAV_RETURN_float_array(msg, s, 4,  4);
}

/**
 * @brief Get field valid from img_moments message
 *
 * @return Message Vadilidity
 */
static inline uint8_t mavlink_msg_img_moments_get_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Decode a img_moments message into a struct
 *
 * @param msg The message to decode
 * @param img_moments C-struct to decode the message contents into
 */
static inline void mavlink_msg_img_moments_decode(const mavlink_message_t* msg, mavlink_img_moments_t* img_moments)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    img_moments->usec = mavlink_msg_img_moments_get_usec(msg);
    mavlink_msg_img_moments_get_s(msg, img_moments->s);
    img_moments->valid = mavlink_msg_img_moments_get_valid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_IMG_MOMENTS_LEN? msg->len : MAVLINK_MSG_ID_IMG_MOMENTS_LEN;
        memset(img_moments, 0, MAVLINK_MSG_ID_IMG_MOMENTS_LEN);
    memcpy(img_moments, _MAV_PAYLOAD(msg), len);
#endif
}
