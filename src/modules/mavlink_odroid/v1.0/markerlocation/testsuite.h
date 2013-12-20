/** @file
 *	@brief MAVLink comm protocol testsuite generated from markerlocation.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef MARKERLOCATION_TESTSUITE_H
#define MARKERLOCATION_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_markerlocation(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_markerlocation(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_marker_location(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_marker_location_t packet_in = {
		93372036854775807ULL,
	}{ 73.0, 74.0, 75.0 },
	}157.0,
	}185.0,
	}213.0,
	}241.0,
	}269.0,
	}125,
	};
	mavlink_marker_location_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.pan = packet_in.pan;
        	packet1.tilt = packet_in.tilt;
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.marker_id = packet_in.marker_id;
        
        	mav_array_memcpy(packet1.pos_xyz, packet_in.pos_xyz, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_location_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_marker_location_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_location_pack(system_id, component_id, &msg , packet1.timestamp , packet1.marker_id , packet1.pos_xyz , packet1.pan , packet1.tilt , packet1.roll , packet1.pitch , packet1.yaw );
	mavlink_msg_marker_location_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_location_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.marker_id , packet1.pos_xyz , packet1.pan , packet1.tilt , packet1.roll , packet1.pitch , packet1.yaw );
	mavlink_msg_marker_location_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_marker_location_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_location_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.marker_id , packet1.pos_xyz , packet1.pan , packet1.tilt , packet1.roll , packet1.pitch , packet1.yaw );
	mavlink_msg_marker_location_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_marker_location_full(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_marker_location_full_t packet_in = {
		93372036854775807ULL,
	}{ 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0, 81.0 },
	}{ 325.0, 326.0, 327.0 },
	}409.0,
	}437.0,
	}465.0,
	}493.0,
	}521.0,
	}233,
	};
	mavlink_marker_location_full_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.pan = packet_in.pan;
        	packet1.tilt = packet_in.tilt;
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.marker_id = packet_in.marker_id;
        
        	mav_array_memcpy(packet1.roation_mat, packet_in.roation_mat, sizeof(float)*9);
        	mav_array_memcpy(packet1.pos_xyz, packet_in.pos_xyz, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_location_full_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_marker_location_full_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_location_full_pack(system_id, component_id, &msg , packet1.timestamp , packet1.marker_id , packet1.roation_mat , packet1.pos_xyz , packet1.pan , packet1.tilt , packet1.roll , packet1.pitch , packet1.yaw );
	mavlink_msg_marker_location_full_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_location_full_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.marker_id , packet1.roation_mat , packet1.pos_xyz , packet1.pan , packet1.tilt , packet1.roll , packet1.pitch , packet1.yaw );
	mavlink_msg_marker_location_full_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_marker_location_full_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_location_full_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.marker_id , packet1.roation_mat , packet1.pos_xyz , packet1.pan , packet1.tilt , packet1.roll , packet1.pitch , packet1.yaw );
	mavlink_msg_marker_location_full_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_markerlocation(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_marker_location(system_id, component_id, last_msg);
	mavlink_test_marker_location_full(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MARKERLOCATION_TESTSUITE_H
