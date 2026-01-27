/*
 * PD Buddy Firmware Library - USB Power Delivery for everyone
 * Copyright 2017-2018 Clayton G. Hobbs
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PDB_FUSB302B_H
#define PDB_FUSB302B_H

#include "pd.h"
#include "pdb_msg.h"

#include <stdbool.h>

#define TICK_IS_64_BIT
#ifdef TICK_IS_64_BIT
#define TICK_TYPE uint64_t
#define TICK_MAX_DELAY 0xFFFFFFFFFFFFFFFF
#else
#define TICK_TYPE uint32_t
#define TICK_MAX_DELAY 0xFFFFFFFF
#endif

typedef bool (*I2CFunc)(const uint8_t deviceAddr, const uint8_t registerAdd, const uint8_t size, uint8_t *buf);
typedef TICK_TYPE (*TimestampFunc)();
typedef void (*DelayFunc)(TICK_TYPE microseconds);

extern I2CFunc I2CRead;
extern I2CFunc I2CWrite;
extern TimestampFunc getTimeStamp;
extern DelayFunc osDelay;

static inline void pd_os_init(TimestampFunc getTimestamp, DelayFunc delayFunc, I2CFunc read, I2CFunc write)
{
	getTimeStamp = getTimestamp;
	osDelay = delayFunc;
	I2CRead = read;
	I2CWrite = write;
}

typedef struct
{
	uint8_t DeviceAddress; // I2C address for this device
} FUSB302;


void fusb_send_message(FUSB302 *fusb, const pd_msg *msg);
bool fusb_rx_pending(FUSB302 *fusb);
/*
* Read a USB Power Delivery message from the FUSB302B
*/
uint8_t fusb_read_message(FUSB302 *fusb, pd_msg *msg);

/*
* Tell the FUSB302B to send a hard reset signal
*/
void fusb_send_hardrst(FUSB302 *fusb);

/*
* FUSB status union
*
* Provides a nicer structure than just an array of uint8_t for working with
* the FUSB302B status and interrupt flags.
*/
typedef union {
	uint8_t bytes[7];
	struct {
		uint8_t status0a;
		uint8_t status1a;
		uint8_t interrupta;
		uint8_t interruptb;
		uint8_t status0;
		uint8_t status1;
		uint8_t interrupt;
	};
} fusb_status;
/*
* Read the FUSB302B status and interrupt flags into *status
*/
bool fusb_get_status(FUSB302 *fusb, fusb_status *status);

/*
* Read the FUSB302B BC_LVL as an enum fusb_typec_current
*/
enum fusb_typec_current fusb_get_typec_current(FUSB302 *fusb);

/*
* Initialization routine for the FUSB302B
*/
bool fusb_setup(FUSB302 *fusb);

/*
* Reset the FUSB302B
*/
bool fusb_reset(FUSB302 *fusb);

bool fusb_read_id(FUSB302 *fusb);

int fusb_runCCLineSelection(FUSB302 *fusb);

int fusb_measureCCLine(FUSB302 *fusb, uint8_t line);

uint8_t fusb_readCCLineMeasurement(FUSB302 *fusb);

int fusb_selectCCLine(FUSB302 *fusb, uint8_t CC1, uint8_t CC2);

// Measure VBus with the MADC and check if its connected
bool fusb_isVBUSConnected(FUSB302 *fusb);

bool fusb_checkVBUS(FUSB302 *fusb, uint16_t minVBUS_mv);

bool fusb_hasCCLineSelection(FUSB302 *fusb);

bool fusb_unplugSink(FUSB302 *fusb);

bool fusb_replugSink(FUSB302 *fusb);

#endif /* PDB_FUSB302B_H */
