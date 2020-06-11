#pragma once
// MESSAGE IMG_POINT PACKING

#define MAVLINK_MSG_ID_IMG_POINT 191

MAVPACKED(
typedef struct __mavlink_img_point_t {
 uint32_t usec; /*< Timestamp (microseconds)*/
 uint16_t y[2]; /*< [u,v] Position (pixels)*/
 uint8_t id; /*< ID of point*/
}) mavlink_img_point_t;

#define MAVLINK_MSG_ID_IMG_POINT_LEN 9
#define MAVLINK_MSG_ID_IMG_POINT_MIN_LEN 9
#define MAVLINK_MSG_ID_191_LEN 9
#define MAVLINK_MSG_ID_191_MIN_LEN 9

#define MAVLINK_MSG_ID_IMG_POINT_CRC 191
#define MAVLINK_MSG_ID_191_CRC 191

#define MAVLINK_MSG_IMG_POINT_FIELD_Y_LEN 2

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_IMG_POINT { \
    191, \
    "IMG_POINT", \
    3, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_img_point_t, usec) }, \
         { "y", NULL, MAVLINK_TYPE_UINT16_T, 2, 4, offsetof(mavlink_img_point_t, y) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_img_point_t, id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_IMG_POINT { \
    "IMG_POINT", \
    3, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_img_point_t, usec) }, \
         { "y", NULL, MAVLINK_TYPE_UINT16_T, 2, 4, offsetof(mavlink_img_point_t, y) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_img_point_t, id) }, \
         } \
}
#endif

/**
 * @brief Pack a img_point message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds)
 * @param id ID of point
 * @param y [u,v] Position (pixels)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_img_point_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t usec, uint8_t id, const uint16_t *y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMG_POINT_LEN];
    _mav_put_uint32_t(buf, 0, usec);
    _mav_put_uint8_t(buf, 8, id);
    _mav_put_uint16_t_array(buf, 4, y, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMG_POINT_LEN);
#else
    mavlink_img_point_t packet;
    packet.usec = usec;
    packet.id = id;
    mav_array_memcpy(packet.y, y, sizeof(uint16_t)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMG_POINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMG_POINT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMG_POINT_MIN_LEN, MAVLINK_MSG_ID_IMG_POINT_LEN, MAVLINK_MSG_ID_IMG_POINT_CRC);
}

/**
 * @brief Pack a img_point message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds)
 * @param id ID of point
 * @param y [u,v] Position (pixels)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_img_point_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t usec,uint8_t id,const uint16_t *y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMG_POINT_LEN];
    _mav_put_uint32_t(buf, 0, usec);
    _mav_put_uint8_t(buf, 8, id);
    _mav_put_uint16_t_array(buf, 4, y, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMG_POINT_LEN);
#else
    mavlink_img_point_t packet;
    packet.usec = usec;
    packet.id = id;
    mav_array_memcpy(packet.y, y, sizeof(uint16_t)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMG_POINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMG_POINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMG_POINT_MIN_LEN, MAVLINK_MSG_ID_IMG_POINT_LEN, MAVLINK_MSG_ID_IMG_POINT_CRC);
}

/**
 * @brief Encode a img_point struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param img_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_img_point_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_img_point_t* img_point)
{
    return mavlink_msg_img_point_pack(system_id, component_id, msg, img_point->usec, img_point->id, img_point->y);
}

/**
 * @brief Encode a img_point struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param img_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_img_point_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_img_point_t* img_point)
{
    return mavlink_msg_img_point_pack_chan(system_id, component_id, chan, msg, img_point->usec, img_point->id, img_point->y);
}

/**
 * @brief Send a img_point message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds)
 * @param id ID of point
 * @param y [u,v] Position (pixels)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_img_point_send(mavlink_channel_t chan, uint32_t usec, uint8_t id, const uint16_t *y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMG_POINT_LEN];
    _mav_put_uint32_t(buf, 0, usec);
    _mav_put_uint8_t(buf, 8, id);
    _mav_put_uint16_t_array(buf, 4, y, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMG_POINT, buf, MAVLINK_MSG_ID_IMG_POINT_MIN_LEN, MAVLINK_MSG_ID_IMG_POINT_LEN, MAVLINK_MSG_ID_IMG_POINT_CRC);
#else
    mavlink_img_point_t packet;
    packet.usec = usec;
    packet.id = id;
    mav_array_memcpy(packet.y, y, sizeof(uint16_t)*2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMG_POINT, (const char *)&packet, MAVLINK_MSG_ID_IMG_POINT_MIN_LEN, MAVLINK_MSG_ID_IMG_POINT_LEN, MAVLINK_MSG_ID_IMG_POINT_CRC);
#endif
}

/**
 * @brief Send a img_point message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_img_point_send_struct(mavlink_channel_t chan, const mavlink_img_point_t* img_point)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_img_point_send(chan, img_point->usec, img_point->id, img_point->y);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMG_POINT, (const char *)img_point, MAVLINK_MSG_ID_IMG_POINT_MIN_LEN, MAVLINK_MSG_ID_IMG_POINT_LEN, MAVLINK_MSG_ID_IMG_POINT_CRC);
#endif
}

#if MAVLINK_MSG_ID_IMG_POINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_img_point_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t usec, uint8_t id, const uint16_t *y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, usec);
    _mav_put_uint8_t(buf, 8, id);
    _mav_put_uint16_t_array(buf, 4, y, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMG_POINT, buf, MAVLINK_MSG_ID_IMG_POINT_MIN_LEN, MAVLINK_MSG_ID_IMG_POINT_LEN, MAVLINK_MSG_ID_IMG_POINT_CRC);
#else
    mavlink_img_point_t *packet = (mavlink_img_point_t *)msgbuf;
    packet->usec = usec;
    packet->id = id;
    mav_array_memcpy(packet->y, y, sizeof(uint16_t)*2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMG_POINT, (const char *)packet, MAVLINK_MSG_ID_IMG_POINT_MIN_LEN, MAVLINK_MSG_ID_IMG_POINT_LEN, MAVLINK_MSG_ID_IMG_POINT_CRC);
#endif
}
#endif

#endif

// MESSAGE IMG_POINT UNPACKING


/**
 * @brief Get field usec from img_point message
 *
 * @return Timestamp (microseconds)
 */
static inline uint32_t mavlink_msg_img_point_get_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field id from img_point message
 *
 * @return ID of point
 */
static inline uint8_t mavlink_msg_img_point_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field y from img_point message
 *
 * @return [u,v] Position (pixels)
 */
static inline uint16_t mavlink_msg_img_point_get_y(const mavlink_message_t* msg, uint16_t *y)
{
    return _MAV_RETURN_uint16_t_array(msg, y, 2,  4);
}

/**
 * @brief Decode a img_point message into a struct
 *
 * @param msg The message to decode
 * @param img_point C-struct to decode the message contents into
 */
static inline void mavlink_msg_img_point_decode(const mavlink_message_t* msg, mavlink_img_point_t* img_point)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    img_point->usec = mavlink_msg_img_point_get_usec(msg);
    mavlink_msg_img_point_get_y(msg, img_point->y);
    img_point->id = mavlink_msg_img_point_get_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_IMG_POINT_LEN? msg->len : MAVLINK_MSG_ID_IMG_POINT_LEN;
        memset(img_point, 0, MAVLINK_MSG_ID_IMG_POINT_LEN);
    memcpy(img_point, _MAV_PAYLOAD(msg), len);
#endif
}
