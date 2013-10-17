/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file interTes.c
 * Testing argument passing to applications.
 */

#include <nuttx/config.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>

#define MAX_ADDR_COUNT 127 /* Maximum number of different sensor addresses */
#define GROUPEND 0x80 /* Bitmask indicating the end of an address group */

__EXPORT int interface_test_main(int argc, char *argv[]);

int interface_test_main(int argc, char *argv[])
{
	int i, addri, addrcount;
	uint8_t * addr;

	if (argc > 3 && (strcmp(argv[2], "-a") == 0 || strcmp(argv[2], "--addrgroups") == 0)) {
		addrcount = atoi(argv[3]);
		if (addrcount <= MAX_ADDR_COUNT) {
			addr = new uint8_t[addrcount];
			for (i = 4; i < argc; i++) {
				if (strcmp(argv[i], ",") == 0) {
					addr[addri] |= GROUPEND;
					continue;
				}

				addr[addri] = (uint8_t) atoi(argv[i]);
				addri++;
			}

			// Last address must be the end of a group in any case.
			addr[addri] |= GROUPEND;
		}
	}

	for (i = 1; i < addrcount; i++) {
		printf("%d", (addr[i] | ~GROUPEND));
		if (addr[i] | GROUPEND) {
			printf("\n");
		}
	}

	return OK;
}
