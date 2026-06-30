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

// Pipe number is local to receiver, but unified
#define NRF_CAMERA_SPECIFIC_PIPE	0
#define NRF_SYNC_BROADCAST_PIPE		1
#define NRF_SYNC_BROADCAST_LEN		8

void nrf_configure_rx(uint8_t cameraAddress[3]);
void nrf_start_rx();

void nrf_configure_tx();
void nrf_tx_powerup();
bool nrf_tx_camera(uint8_t cameraAddress[3], uint8_t *data, uint8_t length);
bool nrf_prepare_broadcast_sync(uint8_t data[NRF_SYNC_BROADCAST_LEN]);
void nrf_tx_trigger();

void nrf_handle_interrupt();

// External
void nrfd_receive_sync_packet(uint8_t sync[NRF_SYNC_BROADCAST_LEN], TimePoint time);
void nrfd_receive_camera_packet(uint8_t *data, uint8_t len, TimePoint time);

#endif

#ifdef __cplusplus
}
#endif

#endif // NRF_SYNC_H