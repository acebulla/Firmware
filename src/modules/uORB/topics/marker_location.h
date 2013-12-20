/*
 * marker_location.h
 *
 *  Created on: Dec 3, 2013
 *      Author: hideki
 */

/**
 * @file marker_location.h
 * Definition of the marker position uORB topic.
 */

#ifndef MARKER_LOCATION_H_
#define MARKER_LOCATION_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * Position of one marker.
 *
 */

struct marker_location_s
{
 uint64_t timestamp;		///< in microseconds, indicates approximately when the measurement was taken.
 float pos_xyz[3]; ///< The xyz vector to the midpoint of the marker.
 float pan; ///< The pan angle.
 float tilt; ///< The tilt angle.
 float pitch; ///< The pitch angle.
 float roll; ///< The roll angle.
 float yaw; ///< The yaw angle.
 uint8_t marker_id; ///< The id of the marker.
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(marker_location);


#endif /* MARKER_LOCATION_H_ */
