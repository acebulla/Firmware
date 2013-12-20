// MESSAGE MARKER_LOCATION_FULL PACKING

#define MAVLINK_MSG_ID_MARKER_LOCATION_FULL 212

typedef struct __mavlink_marker_location_full_t
{
 uint64_t timestamp; ///< A timestamp which indicates when the measurement was taken.
 float roation_mat[9]; ///< The rotationo matrix.
 float pos_xyz[3]; ///< The xyz vector to the midpoint of the marker.
 float pan; ///< The pan angle.
 float tilt; ///< The tilt angle.
 float roll; ///< The roll angle.
 float pitch; ///< The pitch angle.
 float yaw; ///< The yaw angle.
 uint8_t marker_id; ///< The id of the marker.
} mavlink_marker_location_full_t;

#define MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN 77
#define MAVLINK_MSG_ID_212_LEN 77

#define MAVLINK_MSG_ID_MARKER_LOCATION_FULL_CRC 23
#define MAVLINK_MSG_ID_212_CRC 23

#define MAVLINK_MSG_MARKER_LOCATION_FULL_FIELD_ROATION_MAT_LEN 9
#define MAVLINK_MSG_MARKER_LOCATION_FULL_FIELD_POS_XYZ_LEN 3

#define MAVLINK_MESSAGE_INFO_MARKER_LOCATION_FULL { \
	"MARKER_LOCATION_FULL", \
	9, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_marker_location_full_t, timestamp) }, \
         { "roation_mat", NULL, MAVLINK_TYPE_FLOAT, 9, 8, offsetof(mavlink_marker_location_full_t, roation_mat) }, \
         { "pos_xyz", NULL, MAVLINK_TYPE_FLOAT, 3, 44, offsetof(mavlink_marker_location_full_t, pos_xyz) }, \
         { "pan", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_marker_location_full_t, pan) }, \
         { "tilt", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_marker_location_full_t, tilt) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_marker_location_full_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_marker_location_full_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_marker_location_full_t, yaw) }, \
         { "marker_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 76, offsetof(mavlink_marker_location_full_t, marker_id) }, \
         } \
}


/**
 * @brief Pack a marker_location_full message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp A timestamp which indicates when the measurement was taken.
 * @param marker_id The id of the marker.
 * @param roation_mat The rotationo matrix.
 * @param pos_xyz The xyz vector to the midpoint of the marker.
 * @param pan The pan angle.
 * @param tilt The tilt angle.
 * @param roll The roll angle.
 * @param pitch The pitch angle.
 * @param yaw The yaw angle.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_marker_location_full_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, uint8_t marker_id, const float *roation_mat, const float *pos_xyz, float pan, float tilt, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 56, pan);
	_mav_put_float(buf, 60, tilt);
	_mav_put_float(buf, 64, roll);
	_mav_put_float(buf, 68, pitch);
	_mav_put_float(buf, 72, yaw);
	_mav_put_uint8_t(buf, 76, marker_id);
	_mav_put_float_array(buf, 8, roation_mat, 9);
	_mav_put_float_array(buf, 44, pos_xyz, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN);
#else
	mavlink_marker_location_full_t packet;
	packet.timestamp = timestamp;
	packet.pan = pan;
	packet.tilt = tilt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
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
 * @param timestamp A timestamp which indicates when the measurement was taken.
 * @param marker_id The id of the marker.
 * @param roation_mat The rotationo matrix.
 * @param pos_xyz The xyz vector to the midpoint of the marker.
 * @param pan The pan angle.
 * @param tilt The tilt angle.
 * @param roll The roll angle.
 * @param pitch The pitch angle.
 * @param yaw The yaw angle.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_marker_location_full_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,uint8_t marker_id,const float *roation_mat,const float *pos_xyz,float pan,float tilt,float roll,float pitch,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 56, pan);
	_mav_put_float(buf, 60, tilt);
	_mav_put_float(buf, 64, roll);
	_mav_put_float(buf, 68, pitch);
	_mav_put_float(buf, 72, yaw);
	_mav_put_uint8_t(buf, 76, marker_id);
	_mav_put_float_array(buf, 8, roation_mat, 9);
	_mav_put_float_array(buf, 44, pos_xyz, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN);
#else
	mavlink_marker_location_full_t packet;
	packet.timestamp = timestamp;
	packet.pan = pan;
	packet.tilt = tilt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
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
	return mavlink_msg_marker_location_full_pack(system_id, component_id, msg, marker_location_full->timestamp, marker_location_full->marker_id, marker_location_full->roation_mat, marker_location_full->pos_xyz, marker_location_full->pan, marker_location_full->tilt, marker_location_full->roll, marker_location_full->pitch, marker_location_full->yaw);
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
	return mavlink_msg_marker_location_full_pack_chan(system_id, component_id, chan, msg, marker_location_full->timestamp, marker_location_full->marker_id, marker_location_full->roation_mat, marker_location_full->pos_xyz, marker_location_full->pan, marker_location_full->tilt, marker_location_full->roll, marker_location_full->pitch, marker_location_full->yaw);
}

/**
 * @brief Send a marker_location_full message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp A timestamp which indicates when the measurement was taken.
 * @param marker_id The id of the marker.
 * @param roation_mat The rotationo matrix.
 * @param pos_xyz The xyz vector to the midpoint of the marker.
 * @param pan The pan angle.
 * @param tilt The tilt angle.
 * @param roll The roll angle.
 * @param pitch The pitch angle.
 * @param yaw The yaw angle.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_marker_location_full_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t marker_id, const float *roation_mat, const float *pos_xyz, float pan, float tilt, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 56, pan);
	_mav_put_float(buf, 60, tilt);
	_mav_put_float(buf, 64, roll);
	_mav_put_float(buf, 68, pitch);
	_mav_put_float(buf, 72, yaw);
	_mav_put_uint8_t(buf, 76, marker_id);
	_mav_put_float_array(buf, 8, roation_mat, 9);
	_mav_put_float_array(buf, 44, pos_xyz, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER_LOCATION_FULL, buf, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER_LOCATION_FULL, buf, MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN);
#endif
#else
	mavlink_marker_location_full_t packet;
	packet.timestamp = timestamp;
	packet.pan = pan;
	packet.tilt = tilt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
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
 * @brief Get field timestamp from marker_location_full message
 *
 * @return A timestamp which indicates when the measurement was taken.
 */
static inline uint64_t mavlink_msg_marker_location_full_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field marker_id from marker_location_full message
 *
 * @return The id of the marker.
 */
static inline uint8_t mavlink_msg_marker_location_full_get_marker_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  76);
}

/**
 * @brief Get field roation_mat from marker_location_full message
 *
 * @return The rotationo matrix.
 */
static inline uint16_t mavlink_msg_marker_location_full_get_roation_mat(const mavlink_message_t* msg, float *roation_mat)
{
	return _MAV_RETURN_float_array(msg, roation_mat, 9,  8);
}

/**
 * @brief Get field pos_xyz from marker_location_full message
 *
 * @return The xyz vector to the midpoint of the marker.
 */
static inline uint16_t mavlink_msg_marker_location_full_get_pos_xyz(const mavlink_message_t* msg, float *pos_xyz)
{
	return _MAV_RETURN_float_array(msg, pos_xyz, 3,  44);
}

/**
 * @brief Get field pan from marker_location_full message
 *
 * @return The pan angle.
 */
static inline float mavlink_msg_marker_location_full_get_pan(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field tilt from marker_location_full message
 *
 * @return The tilt angle.
 */
static inline float mavlink_msg_marker_location_full_get_tilt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field roll from marker_location_full message
 *
 * @return The roll angle.
 */
static inline float mavlink_msg_marker_location_full_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field pitch from marker_location_full message
 *
 * @return The pitch angle.
 */
static inline float mavlink_msg_marker_location_full_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field yaw from marker_location_full message
 *
 * @return The yaw angle.
 */
static inline float mavlink_msg_marker_location_full_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  72);
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
	marker_location_full->timestamp = mavlink_msg_marker_location_full_get_timestamp(msg);
	mavlink_msg_marker_location_full_get_roation_mat(msg, marker_location_full->roation_mat);
	mavlink_msg_marker_location_full_get_pos_xyz(msg, marker_location_full->pos_xyz);
	marker_location_full->pan = mavlink_msg_marker_location_full_get_pan(msg);
	marker_location_full->tilt = mavlink_msg_marker_location_full_get_tilt(msg);
	marker_location_full->roll = mavlink_msg_marker_location_full_get_roll(msg);
	marker_location_full->pitch = mavlink_msg_marker_location_full_get_pitch(msg);
	marker_location_full->yaw = mavlink_msg_marker_location_full_get_yaw(msg);
	marker_location_full->marker_id = mavlink_msg_marker_location_full_get_marker_id(msg);
#else
	memcpy(marker_location_full, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MARKER_LOCATION_FULL_LEN);
#endif
}
