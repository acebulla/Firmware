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
		{ 17.0, 18.0, 19.0 },
	}101.0,
	}129.0,
	}65,
	};
	mavlink_marker_location_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.pan = packet_in.pan;
        	packet1.tilt = packet_in.tilt;
        	packet1.marker_id = packet_in.marker_id;
        
        	mav_array_memcpy(packet1.pos_xyz, packet_in.pos_xyz, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_location_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_marker_location_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_location_pack(system_id, component_id, &msg , packet1.marker_id , packet1.pos_xyz , packet1.pan , packet1.tilt );
	mavlink_msg_marker_location_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_location_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.marker_id , packet1.pos_xyz , packet1.pan , packet1.tilt );
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
	mavlink_msg_marker_location_send(MAVLINK_COMM_1 , packet1.marker_id , packet1.pos_xyz , packet1.pan , packet1.tilt );
	mavlink_msg_marker_location_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_marker_location_full(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_marker_location_full_t packet_in = {
		{ 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0 },
	}{ 269.0, 270.0, 271.0 },
	}353.0,
	}381.0,
	}173,
	};
	mavlink_marker_location_full_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.pan = packet_in.pan;
        	packet1.tilt = packet_in.tilt;
        	packet1.marker_id = packet_in.marker_id;
        
        	mav_array_memcpy(packet1.roation_mat, packet_in.roation_mat, sizeof(float)*9);
        	mav_array_memcpy(packet1.pos_xyz, packet_in.pos_xyz, sizeof(float)*3);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_location_full_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_marker_location_full_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_location_full_pack(system_id, component_id, &msg , packet1.marker_id , packet1.roation_mat , packet1.pos_xyz , packet1.pan , packet1.tilt );
	mavlink_msg_marker_location_full_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_location_full_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.marker_id , packet1.roation_mat , packet1.pos_xyz , packet1.pan , packet1.tilt );
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
	mavlink_msg_marker_location_full_send(MAVLINK_COMM_1 , packet1.marker_id , packet1.roation_mat , packet1.pos_xyz , packet1.pan , packet1.tilt );
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
