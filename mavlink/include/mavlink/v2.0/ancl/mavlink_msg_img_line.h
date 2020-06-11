#pragma once
// MESSAGE IMG_LINE PACKING

#define MAVLINK_MSG_ID_IMG_LINE 192

MAVPACKED(
typedef struct __mavlink_img_line_t {
 uint32_t usec; /*< Timestamp (microseconds)*/
 float y[2]; /*< [rho,alpha] Line (pixels,radians)*/
 uint8_t id; /*< ID of line*/
}) mavlink_img_line_t;

#define MAVLINK_MSG_ID_IMG_LINE_LEN 13
#define MAVLINK_MSG_ID_IMG_LINE_MIN_LEN 13
#define MAVLINK_MSG_ID_192_LEN 13
#define MAVLINK_MSG_ID_192_MIN_LEN 13

#define MAVLINK_MSG_ID_IMG_LINE_CRC 4
#define MAVLINK_MSG_ID_192_CRC 4

#define MAVLINK_MSG_IMG_LINE_FIELD_Y_LEN 2

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_IMG_LINE { \
    192, \
    "IMG_LINE", \
    3, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_img_line_t, usec) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 2, 4, offsetof(mavlink_img_line_t, y) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_img_line_t, id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_IMG_LINE { \
    "IMG_LINE", \
    3, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_img_line_t, usec) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 2, 4, offsetof(mavlink_img_line_t, y) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_img_line_t, id) }, \
         } \
}
#endif

/**
 * @brief Pack a img_line message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds)
 * @param id ID of line
 * @param y [rho,alpha] Line (pixels,radians)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_img_line_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t usec, uint8_t id, const float *y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMG_LINE_LEN];
    _mav_put_uint32_t(buf, 0, usec);
    _mav_put_uint8_t(buf, 12, id);
    _mav_put_float_array(buf, 4, y, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMG_LINE_LEN);
#else
    mavlink_img_line_t packet;
    packet.usec = usec;
    packet.id = id;
    mav_array_memcpy(packet.y, y, sizeof(float)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMG_LINE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMG_LINE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMG_LINE_MIN_LEN, MAVLINK_MSG_ID_IMG_LINE_LEN, MAVLINK_MSG_ID_IMG_LINE_CRC);
}

/**
 * @brief Pack a img_line message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds)
 * @param id ID of line
 * @param y [rho,alpha] Line (pixels,radians)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_img_line_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t usec,uint8_t id,const float *y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMG_LINE_LEN];
    _mav_put_uint32_t(buf, 0, usec);
    _mav_put_uint8_t(buf, 12, id);
    _mav_put_float_array(buf, 4, y, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMG_LINE_LEN);
#else
    mavlink_img_line_t packet;
    packet.usec = usec;
    packet.id = id;
    mav_array_memcpy(packet.y, y, sizeof(float)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMG_LINE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMG_LINE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMG_LINE_MIN_LEN, MAVLINK_MSG_ID_IMG_LINE_LEN, MAVLINK_MSG_ID_IMG_LINE_CRC);
}

/**
 * @brief Encode a img_line struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param img_line C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_img_line_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_img_line_t* img_line)
{
    return mavlink_msg_img_line_pack(system_id, component_id, msg, img_line->usec, img_line->id, img_line->y);
}

/**
 * @brief Encode a img_line struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param img_line C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_img_line_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_img_line_t* img_line)
{
    return mavlink_msg_img_line_pack_chan(system_id, component_id, chan, msg, img_line->usec, img_line->id, img_line->y);
}

/**
 * @brief Send a img_line message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds)
 * @param id ID of line
 * @param y [rho,alpha] Line (pixels,radians)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_img_line_send(mavlink_channel_t chan, uint32_t usec, uint8_t id, const float *y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMG_LINE_LEN];
    _mav_put_uint32_t(buf, 0, usec);
    _mav_put_uint8_t(buf, 12, id);
    _mav_put_float_array(buf, 4, y, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMG_LINE, buf, MAVLINK_MSG_ID_IMG_LINE_MIN_LEN, MAVLINK_MSG_ID_IMG_LINE_LEN, MAVLINK_MSG_ID_IMG_LINE_CRC);
#else
    mavlink_img_line_t packet;
    packet.usec = usec;
    packet.id = id;
    mav_array_memcpy(packet.y, y, sizeof(float)*2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMG_LINE, (const char *)&packet, MAVLINK_MSG_ID_IMG_LINE_MIN_LEN, MAVLINK_MSG_ID_IMG_LINE_LEN, MAVLINK_MSG_ID_IMG_LINE_CRC);
#endif
}

/**
 * @brief Send a img_line message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_img_line_send_struct(mavlink_channel_t chan, const mavlink_img_line_t* img_line)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_img_line_send(chan, img_line->usec, img_line->id, img_line->y);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMG_LINE, (const char *)img_line, MAVLINK_MSG_ID_IMG_LINE_MIN_LEN, MAVLINK_MSG_ID_IMG_LINE_LEN, MAVLINK_MSG_ID_IMG_LINE_CRC);
#endif
}

#if MAVLINK_MSG_ID_IMG_LINE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_img_line_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t usec, uint8_t id, const float *y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, usec);
    _mav_put_uint8_t(buf, 12, id);
    _mav_put_float_array(buf, 4, y, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMG_LINE, buf, MAVLINK_MSG_ID_IMG_LINE_MIN_LEN, MAVLINK_MSG_ID_IMG_LINE_LEN, MAVLINK_MSG_ID_IMG_LINE_CRC);
#else
    mavlink_img_line_t *packet = (mavlink_img_line_t *)msgbuf;
    packet->usec = usec;
    packet->id = id;
    mav_array_memcpy(packet->y, y, sizeof(float)*2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMG_LINE, (const char *)packet, MAVLINK_MSG_ID_IMG_LINE_MIN_LEN, MAVLINK_MSG_ID_IMG_LINE_LEN, MAVLINK_MSG_ID_IMG_LINE_CRC);
#endif
}
#endif

#endif

// MESSAGE IMG_LINE UNPACKING


/**
 * @brief Get field usec from img_line message
 *
 * @return Timestamp (microseconds)
 */
static inline uint32_t mavlink_msg_img_line_get_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field id from img_line message
 *
 * @return ID of line
 */
static inline uint8_t mavlink_msg_img_line_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field y from img_line message
 *
 * @return [rho,alpha] Line (pixels,radians)
 */
static inline uint16_t mavlink_msg_img_line_get_y(const mavlink_message_t* msg, float *y)
{
    return _MAV_RETURN_float_array(msg, y, 2,  4);
}

/**
 * @brief Decode a img_line message into a struct
 *
 * @param msg The message to decode
 * @param img_line C-struct to decode the message contents into
 */
static inline void mavlink_msg_img_line_decode(const mavlink_message_t* msg, mavlink_img_line_t* img_line)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    img_line->usec = mavlink_msg_img_line_get_usec(msg);
    mavlink_msg_img_line_get_y(msg, img_line->y);
    img_line->id = mavlink_msg_img_line_get_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_IMG_LINE_LEN? msg->len : MAVLINK_MSG_ID_IMG_LINE_LEN;
        memset(img_line, 0, MAVLINK_MSG_ID_IMG_LINE_LEN);
    memcpy(img_line, _MAV_PAYLOAD(msg), len);
#endif
}
