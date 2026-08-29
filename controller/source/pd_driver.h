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

#ifndef PD_DRIVER_H
#define PD_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

void pd_init();
void pd_poll();
void pd_handleOne();
void pd_handleAll();
void pd_renegotiate();

#ifdef __cplusplus
}
#endif

#endif
