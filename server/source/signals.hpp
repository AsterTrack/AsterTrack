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

#ifndef SERVER_SIGNALS_H
#define SERVER_SIGNALS_H

#include "imu/imu.hpp"
#include "target/target.hpp"

/**
 * Signals to Server
 * (from Pipeline, UI can interact directly with Server)
 */

void SignalTrackerDetected(int trackerID);									// Signal: Pipeline -> Server
void SignalTargetCalibUpdate(int trackerID, TargetCalibration3D calib);		// Signal: Pipeline -> Server
void SignalIMUCalibUpdate(int trackerID, IMUIdent ident, IMUCalib calib);	// Signal: Pipeline -> Server
void SignalCameraCalibUpdate(std::vector<CameraCalib> calibs);				// Signal: Pipeline -> Server

#endif // SERVER_SIGNALS_H