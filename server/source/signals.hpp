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

#include "util/error.hpp"
#include "util/eigendef.hpp"

// Forward-declared opaque structs
struct TargetCalibration3D; // target/target.hpp
struct IMUIdent; // imu/imu.hpp
struct IMUCalib; // imu/imu.hpp
struct FrameRecord; // pipeline/record.hpp
struct TrackerRecord; // pipeline/record.hpp
struct TrackerFilter; // tracking/tracking.hpp
struct TrackerInertial; // tracking/tracking.hpp

/**
 * Signals to Server
 * (from Pipeline, UI can interact directly with Server)
 */

void SignalErrorToUser(ErrorMessage error);
void SignalTargetCalibUpdate(int trackerID, TargetCalibration3D calib);
void SignalIMUCalibUpdate(int trackerID, IMUIdent ident, IMUCalib calib);
void SignalCameraCalibUpdate(std::vector<CameraCalib> calibs);
void SignalTrackerDetected(int trackerID);
void SignalTrackerTracked(const FrameRecord &frame, const TrackerRecord &record, const TrackerFilter &filter, const TrackerInertial &inertial);

#endif // SERVER_SIGNALS_H