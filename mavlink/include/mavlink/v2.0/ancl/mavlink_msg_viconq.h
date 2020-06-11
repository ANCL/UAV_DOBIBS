#pragma once
// MESSAGE VICONQ PACKING

#define MAVLINK_MSG_ID_VICONQ 180

MAVPACKED(
typedef struct __mavlink_viconq_t {
 uint32_t usec; /*< Timestamp (microseconds)*/
 int16_t p[3]; /*< Global position (x,y,z): 0.1cm -32.768->32.768 (m)*/
 int16_t v[3]; /*< Global velocity (vx,vy,vz): 0.1cm/s -32.768->32.768 (m/s)*/
 int16_t q[4]; /*< Attitude Quaternion (q0 is real):*20000,-1.63->1.63*/
}) mavlink_viconq_t;

#define MAVLINK_MSG_ID_VICONQ_LEN 24
#define MAVLINK_MSG_ID_VICONQ_MIN_LEN 24
#define MAVLINK_MSG_ID_180_LEN 24
#define MAVLINK_MSG_ID_180_MIN_LEN 24

#define MAVLINK_MSG_ID_VICONQ_CRC 84
#define MAVLINK_MSG_ID_180_CRC 84

#define MAVLINK_MSG_VICONQ_FIELD_P_LEN 3
#define MAVLINK_MSG_VICONQ_FIELD_V_LEN 3
#define MAVLINK_MSG_VICONQ_FIELD_Q_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VICONQ { \
    180, \
    "VICONQ", \
    4, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_viconq_t, usec) }, \
         { "p", NULL, MAVLINK_TYPE_INT16_T, 3, 4, offsetof(mavlink_viconq_t, p) }, \
         { "v", NULL, MAVLINK_TYPE_INT16_T, 3, 10, offsetof(mavlink_viconq_t, v) }, \
         { "q", NULL, MAVLINK_TYPE_INT16_T, 4, 16, offsetof(mavlink_viconq_t, q) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VICONQ { \
    "VICONQ", \
    4, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_viconq_t, usec) }, \
         { "p", NULL, MAVLINK_TYPE_INT16_T, 3, 4, offsetof(mavlink_viconq_t, p) }, \
         { "v", NULL, MAVLINK_TYPE_INT16_T, 3, 10, offsetof(mavlink_viconq_t, v) }, \
         { "q", NULL, MAVLINK_TYPE_INT16_T, 4, 16, offsetof(mavlink_viconq_t, q) }, \
         } \
}
#endif

/**
 * @brief Pack a viconq message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds)
 * @param p Global position (x,y,z): 0.1cm -32.768->32.768 (m)
 * @param v Global velocity (vx,vy,vz): 0.1cm/s -32.768->32.768 (m/s)
 * @param q Attitude Quaternion (q0 is real):*20000,-1.63->1.63
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_viconq_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t usec, const int16_t *p, const int16_t *v, const int16_t *q)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VICONQ_LEN];
    _mav_put_uint32_t(buf, 0, usec);
    _mav_put_int16_t_array(buf, 4, p, 3);
    _mav_put_int16_t_array(buf, 10, v, 3);
    _mav_put_int16_t_array(buf, 16, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VICONQ_LEN);
#else
    mavlink_viconq_t packet;
    packet.usec = usec;
    mav_array_memcpy(packet.p, p, sizeof(int16_t)*3);
    mav_array_memcpy(packet.v, v, sizeof(int16_t)*3);
    mav_array_memcpy(packet.q, q, sizeof(int16_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VICONQ_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VICONQ;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VICONQ_MIN_LEN, MAVLINK_MSG_ID_VICONQ_LEN, MAVLINK_MSG_ID_VICONQ_CRC);
}

/**
 * @brief Pack a viconq message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds)
 * @param p Global position (x,y,z): 0.1cm -32.768->32.768 (m)
 * @param v Global velocity (vx,vy,vz): 0.1cm/s -32.768->32.768 (m/s)
 * @param q Attitude Quaternion (q0 is real):*20000,-1.63->1.63
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_viconq_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t usec,const int16_t *p,const int16_t *v,const int16_t *q)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VICONQ_LEN];
    _mav_put_uint32_t(buf, 0, usec);
    _mav_put_int16_t_array(buf, 4, p, 3);
    _mav_put_int16_t_array(buf, 10, v, 3);
    _mav_put_int16_t_array(buf, 16, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VICONQ_LEN);
#else
    mavlink_viconq_t packet;
    packet.usec = usec;
    mav_array_memcpy(packet.p, p, sizeof(int16_t)*3);
    mav_array_memcpy(packet.v, v, sizeof(int16_t)*3);
    mav_array_memcpy(packet.q, q, sizeof(int16_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VICONQ_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VICONQ;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VICONQ_MIN_LEN, MAVLINK_MSG_ID_VICONQ_LEN, MAVLINK_MSG_ID_VICONQ_CRC);
}

/**
 * @brief Encode a viconq struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param viconq C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_viconq_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_viconq_t* viconq)
{
    return mavlink_msg_viconq_pack(system_id, component_id, msg, viconq->usec, viconq->p, viconq->v, viconq->q);
}

/**
 * @brief Encode a viconq struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param viconq C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_viconq_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_viconq_t* viconq)
{
    return mavlink_msg_viconq_pack_chan(system_id, component_id, chan, msg, viconq->usec, viconq->p, viconq->v, viconq->q);
}

/**
 * @brief Send a viconq message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds)
 * @param p Global position (x,y,z): 0.1cm -32.768->32.768 (m)
 * @param v Global velocity (vx,vy,vz): 0.1cm/s -32.768->32.768 (m/s)
 * @param q Attitude Quaternion (q0 is real):*20000,-1.63->1.63
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_viconq_send(mavlink_channel_t chan, uint32_t usec, const int16_t *p, const int16_t *v, const int16_t *q)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VICONQ_LEN];
    _mav_put_uint32_t(buf, 0, usec);
    _mav_put_int16_t_array(buf, 4, p, 3);
    _mav_put_int16_t_array(buf, 10, v, 3);
    _mav_put_int16_t_array(buf, 16, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VICONQ, buf, MAVLINK_MSG_ID_VICONQ_MIN_LEN, MAVLINK_MSG_ID_VICONQ_LEN, MAVLINK_MSG_ID_VICONQ_CRC);
#else
    mavlink_viconq_t packet;
    packet.usec = usec;
    mav_array_memcpy(packet.p, p, sizeof(int16_t)*3);
    mav_array_memcpy(packet.v, v, sizeof(int16_t)*3);
    mav_array_memcpy(packet.q, q, sizeof(int16_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VICONQ, (const char *)&packet, MAVLINK_MSG_ID_VICONQ_MIN_LEN, MAVLINK_MSG_ID_VICONQ_LEN, MAVLINK_MSG_ID_VICONQ_CRC);
#endif
}

/**
 * @brief Send a viconq message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_viconq_send_struct(mavlink_channel_t chan, const mavlink_viconq_t* viconq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_viconq_send(chan, viconq->usec, viconq->p, viconq->v, viconq->q);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VICONQ, (const char *)viconq, MAVLINK_MSG_ID_VICONQ_MIN_LEN, MAVLINK_MSG_ID_VICONQ_LEN, MAVLINK_MSG_ID_VICONQ_CRC);
#endif
}

#if MAVLINK_MSG_ID_VICONQ_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_viconq_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t usec, const int16_t *p, const int16_t *v, const int16_t *q)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, usec);
    _mav_put_int16_t_array(buf, 4, p, 3);
    _mav_put_int16_t_array(buf, 10, v, 3);
    _mav_put_int16_t_array(buf, 16, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VICONQ, buf, MAVLINK_MSG_ID_VICONQ_MIN_LEN, MAVLINK_MSG_ID_VICONQ_LEN, MAVLINK_MSG_ID_VICONQ_CRC);
#else
    mavlink_viconq_t *packet = (mavlink_viconq_t *)msgbuf;
    packet->usec = usec;
    mav_array_memcpy(packet->p, p, sizeof(int16_t)*3);
    mav_array_memcpy(packet->v, v, sizeof(int16_t)*3);
    mav_array_memcpy(packet->q, q, sizeof(int16_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VICONQ, (const char *)packet, MAVLINK_MSG_ID_VICONQ_MIN_LEN, MAVLINK_MSG_ID_VICONQ_LEN, MAVLINK_MSG_ID_VICONQ_CRC);
#endif
}
#endif

#endif

// MESSAGE VICONQ UNPACKING


/**
 * @brief Get field usec from viconq message
 *
 * @return Timestamp (microseconds)
 */
static inline uint32_t mavlink_msg_viconq_get_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field p from viconq message
 *
 * @return Global position (x,y,z): 0.1cm -32.768->32.768 (m)
 */
static inline uint16_t mavlink_msg_viconq_get_p(const mavlink_message_t* msg, int16_t *p)
{
    return _MAV_RETURN_int16_t_array(msg, p, 3,  4);
}

/**
 * @brief Get field v from viconq message
 *
 * @return Global velocity (vx,vy,vz): 0.1cm/s -32.768->32.768 (m/s)
 */
static inline uint16_t mavlink_msg_viconq_get_v(const mavlink_message_t* msg, int16_t *v)
{
    return _MAV_RETURN_int16_t_array(msg, v, 3,  10);
}

/**
 * @brief Get field q from viconq message
 *
 * @return Attitude Quaternion (q0 is real):*20000,-1.63->1.63
 */
static inline uint16_t mavlink_msg_viconq_get_q(const mavlink_message_t* msg, int16_t *q)
{
    return _MAV_RETURN_int16_t_array(msg, q, 4,  16);
}

/**
 * @brief Decode a viconq message into a struct
 *
 * @param msg The message to decode
 * @param viconq C-struct to decode the message contents into
 */
static inline void mavlink_msg_viconq_decode(const mavlink_message_t* msg, mavlink_viconq_t* viconq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    viconq->usec = mavlink_msg_viconq_get_usec(msg);
    mavlink_msg_viconq_get_p(msg, viconq->p);
    mavlink_msg_viconq_get_v(msg, viconq->v);
    mavlink_msg_viconq_get_q(msg, viconq->q);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VICONQ_LEN? msg->len : MAVLINK_MSG_ID_VICONQ_LEN;
        memset(viconq, 0, MAVLINK_MSG_ID_VICONQ_LEN);
    memcpy(viconq, _MAV_PAYLOAD(msg), len);
#endif
}
