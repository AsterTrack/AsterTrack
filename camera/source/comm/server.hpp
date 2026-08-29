/**
AsterTrack Optical Tracking System
Copyright (C) 2026 Seneral <seneral@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, specifically version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef SERVER_H
#define SERVER_H

#include <string>
#include <cstdint>

void *server_init(std::string host, std::string port);

void server_deinit(void **portPtr);

bool server_start(void *port);

void server_stop(void *port);

int server_wait(void *port, uint32_t timeoutUS);

int server_read(void *port, uint8_t *buf, uint32_t len);

int server_write(void *port, const uint8_t *buf, uint32_t len);

void server_submit(void *port);

void server_flush(void *port);

int server_getTXQueue(void *port);

#endif // SERVER_H