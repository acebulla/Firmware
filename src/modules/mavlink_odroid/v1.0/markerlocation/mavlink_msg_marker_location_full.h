// MESSAGE MARKER_LOCATION_FULL PACKING

#define MAVLINK_MSG_ID_MARKER_LOCATION_FULL 212

typedef struct __mavlink_marker_location_full_t
{
 float roation_mat[9]; ///< The rotationo matrix.
 float pos_xyz[3]; ///< The xyz vector to the midpoint of the marker.
 float pan; ///< The pan angle.
 float tilt; ///< The tilt angle.
 uint8_t marker_id; ///< The id of the marker.
} mavlink_marker_location_full_t;

#define MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN 57
#define MAVLINK_MSG_ID_212_LEN 57

#define MAVLINK_MSG_ID_MARKER_LOCATION_FULL_CRC 82
#define MAVLINK_MSG_ID_212_CRC 82

#define MAVLINK_MSG_MARKER_LOCATION_FULL_FIELD_ROATION_MAT_LEN 9
#define MAVLINK_MSG_MARKER_LOCATION_FULL_FIELD_POS_XYZ_LEN 3

#define MAVLINK_MESSAGE_INFO_MARKER_LOCATION_FULL { \
	"MARKER_LOCATION_FULL", \
	5, \
	{  { "roation_mat", NULL, MAVLINK_TYPE_FLOAT, 9, 0, offsetof(mavlink_marker_location_full_t, roation_mat) }, \
         { "pos_xyz", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_marker_location_full_t, pos_xyz) }, \
         { "pan", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_marker_location_full_t, pan) }, \
         { "tilt", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_marker_location_full_t, tilt) }, \
         { "marker_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_marker_location_full_t, marker_id) }, \
         } \
}


/**
 * @brief Pack a marker_location_full message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param marker_id The id of the marker.
 * @param roation_mat The rotationo matrix.
 * @param pos_xyz The xyz vector to the midpoint of the marker.
 * @param pan The pan angle.
 * @param tilt The tilt angle.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_marker_location_full_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t marker_id, const float *roation_mat, const float *pos_xyz, float pan, float tilt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN];
	_mav_put_float(buf, 48, pan);
	_mav_put_float(buf, 52, tilt);
	_mav_put_uint8_t(buf, 56, marker_id);
	_mav_put_float_array(buf, 0, roation_mat, 9);
	_mav_put_float_array(buf, 36, pos_xyz, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN);
#else
	mavlink_marker_location_full_t packet;
	packet.pan = pan;
	packet.tilt = tilt;
	packet.marker_id = marker_id;
	mav_array_memcpy(packet.roation_mat, roation_mat, sizeof(float)*9);
	mav_array_memcpy(packet.pos_xyz, pos_xyz, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MARKER_LOCATION_FULL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN);
#endif
}

/**
 * @brief Pack a marker_location_full message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param marker_id The id of the marker.
 * @param roation_mat The rotationo matrix.
 * @param pos_xyz The xyz vector to the midpoint of the marker.
 * @param pan The pan angle.
 * @param tilt The tilt angle.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_marker_location_full_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t marker_id,const float *roation_mat,const float *pos_xyz,float pan,float tilt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN];
	_mav_put_float(buf, 48, pan);
	_mav_put_float(buf, 52, tilt);
	_mav_put_uint8_t(buf, 56, marker_id);
	_mav_put_float_array(buf, 0, roation_mat, 9);
	_mav_put_float_array(buf, 36, pos_xyz, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN);
#else
	mavlink_marker_location_full_t packet;
	packet.pan = pan;
	packet.tilt = tilt;
	packet.marker_id = marker_id;
	mav_array_memcpy(packet.roation_mat, roation_mat, sizeof(float)*9);
	mav_array_memcpy(packet.pos_xyz, pos_xyz, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MARKER_LOCATION_FULL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN);
#endif
}

/**
 * @brief Encode a marker_location_full struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param marker_location_full C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_marker_location_full_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_marker_location_full_t* marker_location_full)
{
	return mavlink_msg_marker_location_full_pack(system_id, component_id, msg, marker_location_full->marker_id, marker_location_full->roation_mat, marker_location_full->pos_xyz, marker_location_full->pan, marker_location_full->tilt);
}

/**
 * @brief Encode a marker_location_full struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param marker_location_full C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_marker_location_full_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_marker_location_full_t* marker_location_full)
{
	return mavlink_msg_marker_location_full_pack_chan(system_id, component_id, chan, msg, marker_location_full->marker_id, marker_location_full->roation_mat, marker_location_full->pos_xyz, marker_location_full->pan, marker_location_full->tilt);
}

/**
 * @brief Send a marker_location_full message
 * @param chan MAVLink channel to send the message
 *
 * @param marker_id The id of the marker.
 * @param roation_mat The rotationo matrix.
 * @param pos_xyz The xyz vector to the midpoint of the marker.
 * @param pan The pan angle.
 * @param tilt The tilt angle.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_marker_location_full_send(mavlink_channel_t chan, uint8_t marker_id, const float *roation_mat, const float *pos_xyz, float pan, float tilt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN];
	_mav_put_float(buf, 48, pan);
	_mav_put_float(buf, 52, tilt);
	_mav_put_uint8_t(buf, 56, marker_id);
	_mav_put_float_array(buf, 0, roation_mat, 9);
	_mav_put_float_array(buf, 36, pos_xyz, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER_LOCATION_FULL, buf, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER_LOCATION_FULL, buf, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN);
#endif
#else
	mavlink_marker_location_full_t packet;
	packet.pan = pan;
	packet.tilt = tilt;
	packet.marker_id = marker_id;
	mav_array_memcpy(packet.roation_mat, roation_mat, sizeof(float)*9);
	mav_array_memcpy(packet.pos_xyz, pos_xyz, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER_LOCATION_FULL, (const char *)&packet, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER_LOCATION_FULL, (const char *)&packet, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN);
#endif
#endif
}

#endif

// MESSAGE MARKER_LOCATION_FULL UNPACKING


/**
 * @brief Get field marker_id from marker_location_full message
 *
 * @return The id of the marker.
 */
static inline uint8_t mavlink_msg_marker_location_full_get_marker_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  56);
}

/**
 * @brief Get field roation_mat from marker_location_full message
 *
 * @return The rotationo matrix.
 */
static inline uint16_t mavlink_msg_marker_location_full_get_roation_mat(const mavlink_message_t* msg, float *roation_mat)
{
	return _MAV_RETURN_float_array(msg, roation_mat, 9,  0);
}

/**
 * @brief Get field pos_xyz from marker_location_full message
 *
 * @return The xyz vector to the midpoint of the marker.
 */
static inline uint16_t mavlink_msg_marker_location_full_get_pos_xyz(const mavlink_message_t* msg, float *pos_xyz)
{
	return _MAV_RETURN_float_array(msg, pos_xyz, 3,  36);
}

/**
 * @brief Get field pan from marker_location_full message
 *
 * @return The pan angle.
 */
static inline float mavlink_msg_marker_location_full_get_pan(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field tilt from marker_location_full message
 *
 * @return The tilt angle.
 */
static inline float mavlink_msg_marker_location_full_get_tilt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Decode a marker_location_full message into a struct
 *
 * @param msg The message to decode
 * @param marker_location_full C-struct to decode the message contents into
 */
static inline void mavlink_msg_marker_location_full_decode(const mavlink_message_t* msg, mavlink_marker_location_full_t* marker_location_full)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_marker_location_full_get_roation_mat(msg, marker_location_full->roation_mat);
	mavlink_msg_marker_location_full_get_pos_xyz(msg, marker_location_full->pos_xyz);
	marker_location_full->pan = mavlink_msg_marker_location_full_get_pan(msg);
	marker_location_full->tilt = mavlink_msg_marker_location_full_get_tilt(msg);
	marker_location_full->marker_id = mavlink_msg_marker_location_full_get_marker_id(msg);
#else
	memcpy(marker_location_full, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN);
#endif
}
