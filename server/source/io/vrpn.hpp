/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef VRPN_H
#define VRPN_H

#include "util/eigendef.hpp"
#include "util/util.hpp" // TimePoint_t
#include "util/memory.hpp" // unique_ptr, opaque_ptr

#include "imu/device.hpp"

#include "vrpn_imu.hpp"

#define VRPN_USE_WINSOCK2

#include "vrpn/vrpn_Tracker.h"
#include "vrpn/vrpn_Connection.h"


/*
 * VRPN (Virtual Reality Private Network) interface to exchange tracking data between programs (locally or on the network)
 */

class VRPN_API vrpn_Tracker_AsterTrack : public vrpn_Tracker, vrpn_IMU_Remote
{
	public:
		vrpn_Tracker_AsterTrack(int ID, const char *path, vrpn_Connection *connection, int index = 0);
		~vrpn_Tracker_AsterTrack();

		void updatePose(int sensor, TimePoint_t time, Eigen::Isometry3f pose);

		bool isConnected();

		std::shared_ptr<IMUDevice> remoteIMU;

		virtual void mainloop ();

		int id;
		const std::string path;

	protected:
};

struct vrpn_Tracker_Wrapper
{
	std::string path;
	bool editing = false;
	std::unique_ptr<vrpn_Tracker_Remote> remote = nullptr;
	bool receivedPackets = false;
	TimePoint_t lastPacket, lastTimestamp;
	bool logPackets = true;
	Eigen::Isometry3f pose;

	vrpn_Tracker_Wrapper(std::string Path);

	bool isConnected();

	void connect();
};

std::vector<std::string> vrpn_getKnownTrackers(vrpn_Connection *connection);

#endif /* VRPN_H */
