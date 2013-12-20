// MESSAGE MARKER_LOCATION PACKING

#define MAVLINK_MSG_ID_MARKER_LOCATION 211

typedef struct __mavlink_marker_location_t
{
 uint64_t timestamp; ///< A timestamp which indicates when the measurement was taken.
 float pos_xyz[3]; ///< The xyz vector to the midpoint of the marker.
 float pan; ///< The pan angle.
 float tilt; ///< The tilt angle.
 float roll; ///< The roll angle.
 float pitch; ///< The pitch angle.
 float yaw; ///< The yaw angle.
 uint8_t marker_id; ///< The id of the marker.
} mavlink_marker_location_t;

#define MAVLINK_MSG_ID_MARKER_LOCATION_LEN 41
#define MAVLINK_MSG_ID_211_LEN 41

#define MAVLINK_MSG_ID_MARKER_LOCATION_CRC 246
#define MAVLINK_MSG_ID_211_CRC 246

#define MAVLINK_MSG_MARKER_LOCATION_FIELD_POS_XYZ_LEN 3

#define MAVLINK_MESSAGE_INFO_MARKER_LOCATION { \
	"MARKER_LOCATION", \
	8, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_marker_location_t, timestamp) }, \
         { "pos_xyz", NULL, MAVLINK_TYPE_FLOAT, 3, 8, offsetof(mavlink_marker_location_t, pos_xyz) }, \
         { "pan", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_marker_location_t, pan) }, \
         { "tilt", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_marker_location_t, tilt) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_marker_location_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_marker_location_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_marker_location_t, yaw) }, \
         { "marker_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_marker_location_t, marker_id) }, \
         } \
}


/**
 * @brief Pack a marker_location message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp A timestamp which indicates when the measurement was taken.
 * @param marker_id The id of the marker.
 * @param pos_xyz The xyz vector to the midpoint of the marker.
 * @param pan The pan angle.
 * @param tilt The tilt angle.
 * @param roll The roll angle.
 * @param pitch The pitch angle.
 * @param yaw The yaw angle.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_marker_location_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, uint8_t marker_id, const float *pos_xyz, float pan, float tilt, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MARKER_LOCATION_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 20, pan);
	_mav_put_float(buf, 24, tilt);
	_mav_put_float(buf, 28, roll);
	_mav_put_float(buf, 32, pitch);
	_mav_put_float(buf, 36, yaw);
	_mav_put_uint8_t(buf, 40, marker_id);
	_mav_put_float_array(buf, 8, pos_xyz, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MARKER_LOCATION_LEN);
#else
	mavlink_marker_location_t packet;
	packet.timestamp = timestamp;
	packet.pan = pan;
	packet.tilt = tilt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.marker_id = marker_id;
	mav_array_memcpy(packet.pos_xyz, pos_xyz, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MARKER_LOCATION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MARKER_LOCATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MARKER_LOCATION_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MARKER_LOCATION_LEN);
#endif
}

/**
 * @brief Pack a marker_location message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp A timestamp which indicates when the measurement was taken.
 * @param marker_id The id of the marker.
 * @param pos_xyz The xyz vector to the midpoint of the marker.
 * @param pan The pan angle.
 * @param tilt The tilt angle.
 * @param roll The roll angle.
 * @param pitch The pitch angle.
 * @param yaw The yaw angle.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_marker_location_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,uint8_t marker_id,const float *pos_xyz,float pan,float tilt,float roll,float pitch,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MARKER_LOCATION_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 20, pan);
	_mav_put_float(buf, 24, tilt);
	_mav_put_float(buf, 28, roll);
	_mav_put_float(buf, 32, pitch);
	_mav_put_float(buf, 36, yaw);
	_mav_put_uint8_t(buf, 40, marker_id);
	_mav_put_float_array(buf, 8, pos_xyz, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MARKER_LOCATION_LEN);
#else
	mavlink_marker_location_t packet;
	packet.timestamp = timestamp;
	packet.pan = pan;
	packet.tilt = tilt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.marker_id = marker_id;
	mav_array_memcpy(packet.pos_xyz, pos_xyz, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MARKER_LOCATION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MARKER_LOCATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MARKER_LOCATION_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MARKER_LOCATION_LEN);
#endif
}

/**
 * @brief Encode a marker_location struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param marker_location C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_marker_location_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_marker_location_t* marker_location)
{
	return mavlink_msg_marker_location_pack(system_id, component_id, msg, marker_location->timestamp, marker_location->marker_id, marker_location->pos_xyz, marker_location->pan, marker_location->tilt, marker_location->roll, marker_location->pitch, marker_location->yaw);
}

/**
 * @brief Encode a marker_location struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param marker_location C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_marker_location_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_marker_location_t* marker_location)
{
	return mavlink_msg_marker_location_pack_chan(system_id, component_id, chan, msg, marker_location->timestamp, marker_location->marker_id, marker_location->pos_xyz, marker_location->pan, marker_location->tilt, marker_location->roll, marker_location->pitch, marker_location->yaw);
}

/**
 * @brief Send a marker_location message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp A timestamp which indicates when the measurement was taken.
 * @param marker_id The id of the marker.
 * @param pos_xyz The xyz vector to the midpoint of the marker.
 * @param pan The pan angle.
 * @param tilt The tilt angle.
 * @param roll The roll angle.
 * @param pitch The pitch angle.
 * @param yaw The yaw angle.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_marker_location_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t marker_id, const float *pos_xyz, float pan, float tilt, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MARKER_LOCATION_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 20, pan);
	_mav_put_float(buf, 24, tilt);
	_mav_put_float(buf, 28, roll);
	_mav_put_float(buf, 32, pitch);
	_mav_put_float(buf, 36, yaw);
	_mav_put_uint8_t(buf, 40, marker_id);
	_mav_put_float_array(buf, 8, pos_xyz, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER_LOCATION, buf, MAVLINK_MSG_ID_MARKER_LOCATION_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER_LOCATION, buf, MAVLINK_MSG_ID_MARKER_LOCATION_LEN);
#endif
#else
	mavlink_marker_location_t packet;
	packet.timestamp = timestamp;
	packet.pan = pan;
	packet.tilt = tilt;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.marker_id = marker_id;
	mav_array_memcpy(packet.pos_xyz, pos_xyz, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER_LOCATION, (const char *)&packet, MAVLINK_MSG_ID_MARKER_LOCATION_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER_LOCATION, (const char *)&packet, MAVLINK_MSG_ID_MARKER_LOCATION_LEN);
#endif
#endif
}

#endif

// MESSAGE MARKER_LOCATION UNPACKING


/**
 * @brief Get field timestamp from marker_location message
 *
 * @return A timestamp which indicates when the measurement was taken.
 */
static inline uint64_t mavlink_msg_marker_location_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field marker_id from marker_location message
 *
 * @return The id of the marker.
 */
static inline uint8_t mavlink_msg_marker_location_get_marker_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field pos_xyz from marker_location message
 *
 * @return The xyz vector to the midpoint of the marker.
 */
static inline uint16_t mavlink_msg_marker_location_get_pos_xyz(const mavlink_message_t* msg, float *pos_xyz)
{
	return _MAV_RETURN_float_array(msg, pos_xyz, 3,  8);
}

/**
 * @brief Get field pan from marker_location message
 *
 * @return The pan angle.
 */
static inline float mavlink_msg_marker_location_get_pan(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field tilt from marker_location message
 *
 * @return The tilt angle.
 */
static inline float mavlink_msg_marker_location_get_tilt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field roll from marker_location message
 *
 * @return The roll angle.
 */
static inline float mavlink_msg_marker_location_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field pitch from marker_location message
 *
 * @return The pitch angle.
 */
static inline float mavlink_msg_marker_location_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field yaw from marker_location message
 *
 * @return The yaw angle.
 */
static inline float mavlink_msg_marker_location_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a marker_location message into a struct
 *
 * @param msg The message to decode
 * @param marker_location C-struct to decode the message contents into
 */
static inline void mavlink_msg_marker_location_decode(const mavlink_message_t* msg, mavlink_marker_location_t* marker_location)
{
#if MAVLINK_NEED_BYTE_SWAP
	marker_location->timestamp = mavlink_msg_marker_location_get_timestamp(msg);
	mavlink_msg_marker_location_get_pos_xyz(msg, marker_location->pos_xyz);
	marker_location->pan = mavlink_msg_marker_location_get_pan(msg);
	marker_location->tilt = mavlink_msg_marker_location_get_tilt(msg);
	marker_location->roll = mavlink_msg_marker_location_get_roll(msg);
	marker_location->pitch = mavlink_msg_marker_location_get_pitch(msg);
	marker_location->yaw = mavlink_msg_marker_location_get_yaw(msg);
	marker_location->marker_id = mavlink_msg_marker_location_get_marker_id(msg);
#else
	memcpy(marker_location, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MARKER_LOCATION_LEN);
#endif
}
