/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef NRF_SYNC_H
#define NRF_SYNC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#ifdef USE_SPI_NRF_SYNC

#include "util.h"

// NOTE: Basically NONE of these is safe to call from an interrupt
// They are all synchronous with busy waiting for the SPI to transfer
// And interrupts are non-preeemptive on the STM32G030

/*
Dual-use Camera->Controller and (if required) Controller->Camera pipe
Uses Auto-ACK, may use ACK-Payloads to eliminate dual-use in the future
No Specific use case in mind yet, but only pipe 0 has an independent fully-qualified RX address
*/
#define NRF_DIRECT_PIPE             0

/*
Meta Broadcast:
General packets for cameras, using dynamic length, but no ACK (broadcast)
May include initial setup information without requiring a cable
Or wakeup request to trigger camera SBC startup from standby
*/
#define NRF_META_BROADCAST_PIPE		1

/*
Sync Broadcast:
Fixed rate packets with 32-bit counter, with camera MCU acting as a PLL
Division factor and offset for frame captures are configured per camera
Allows multiple sync groups with same alignment to share the RF channel
Since it triggers a frame capture indirectly, packet drops are no issue
The main reason to send 32-Bit and not 8-Bit is subdivision consistency
Otherwise, any receiver joining later will have an invalid 32-Bit count
*/
#define NRF_SYNC_BROADCAST_PIPE		2
#define NRF_SYNC_BROADCAST_LEN		4

/*
Trig Broadcast:
Camera MCU will immediately trigger frame capture upon RX on this pipe
Pipe may use one of multiple sub-addresses reserved for trig packets
Content is the 8-bit truncated frame ID
*/
#define NRF_TRIG_BROADCAST_PIPE		3
#define NRF_TRIG_BROADCAST_LEN		1

void nrf_setup_camera(uint8_t cameraAddress[3]);
void nrf_setup_sync_base();

void nrf_rx_powerup();

void nrf_tx_powerup();
bool nrf_tx_general(uint8_t cameraAddress[3], uint8_t *data, uint8_t length);
bool nrf_tx_prepare_broadcast_sync(uint8_t data[NRF_SYNC_BROADCAST_LEN]);
void nrf_tx_trigger();

void nrf_handle_interrupt();

// External
void nrfd_receive_sync_packet(uint8_t sync[NRF_SYNC_BROADCAST_LEN], TimePoint time);
void nrfd_receive_trig_packet(uint8_t sync[NRF_TRIG_BROADCAST_LEN], TimePoint time);
void nrfd_receive_camera_packet(uint8_t *data, uint8_t len, TimePoint time);

#endif

#ifdef __cplusplus
}
#endif

#endif // NRF_SYNC_H