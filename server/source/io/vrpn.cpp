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

#include "vrpn.hpp"

#include "vrpn/quat.h"

#include "imu/device.hpp"

#include "util/log.hpp"


/*
 * VRPN (Virtual Reality Private Network) interface to output tracking data to other programs (locally or on the network)
 */


/* Integrating VRPN into a more modern C++ style */

template<> void OpaqueDeleter<vrpn_Connection>::operator()(vrpn_Connection* ptr) const
{ ptr->removeReference(); }


/* Conversion from internally used steady_clock to timeval used by VRPN */

static inline struct timeval createTimestamp(TimePoint_t time)
{
	// Create timestamp without assuming steady_clock to be equivalent to timeval
	struct timeval time_now, timestamp;
	vrpn_gettimeofday(&time_now, NULL);
	TimePoint_t time_ref = sclock::now();
	long diffUS = std::chrono::duration_cast<std::chrono::microseconds>(time_ref - time).count();
	long long timeUS = time_now.tv_sec * 1000000 + time_now.tv_usec;
	timeUS -= diffUS;
	timestamp.tv_sec = timeUS/1000000;
	timestamp.tv_usec = timeUS%1000000;
	LOG(LIO, LTrace, "Time %.2fms ago resulted in timestamp (%ld, %ld) from cur time (%ld, %ld) and diff (%ldus)",
		std::chrono::duration_cast<std::chrono::microseconds>(time_ref - time).count()/1000.0f,
		timestamp.tv_sec, timestamp.tv_usec, time_now.tv_sec, time_now.tv_usec, diffUS)
	return timestamp;
}

static inline TimePoint_t getTimestamp(struct timeval time_msg)
{
	// Recreate timestamp without assuming steady_clock to be equivalent to timeval
	struct timeval time_now;
	vrpn_gettimeofday(&time_now, NULL);
	TimePoint_t timestamp = sclock::now();
	timestamp -= std::chrono::seconds(time_now.tv_sec-time_msg.tv_sec);
	timestamp -= std::chrono::microseconds(time_now.tv_usec-time_msg.tv_usec);
	return timestamp;
}
/* Wrapper for a VRPN tracker output */

static void handleIMURaw(void *data, const vrpn_IMURAWCB t)
{ // Currently not sent by AsterTrack
	vrpn_Tracker_AsterTrack *tracker = (vrpn_Tracker_AsterTrack*)data;

	if (!tracker->remoteIMU)
		tracker->remoteIMU = registerRemoteIMU(tracker->path);
	if (!tracker->remoteIMU)
	{
		LOG(LIO, LWarn, "Tracker %s (%d) is dropping a raw IMU sample!", tracker->path.c_str(), tracker->id);
		return;
	}

	IMUSampleRaw sample;
	sample.timestamp = getTimestamp(t.msg_time);
	sample.accel = Eigen::Vector3d(t.accel[0], t.accel[1], t.accel[2]).cast<float>();
	sample.gyro = Eigen::Vector3d(t.gyro[0], t.gyro[1], t.gyro[2]).cast<float>();
	sample.mag = Eigen::Vector3d(t.mag[0], t.mag[1], t.mag[2]).cast<float>();

	tracker->remoteIMU->isFused = false;
	if (sample.mag.squaredNorm() > 0.001f)
		tracker->remoteIMU->hasMag = true;
	tracker->remoteIMU->samplesRaw.push_back(sample);

	auto provider = getRemoteIMUProvider();
	if (provider->recordLatencyMeasurements)
	{ // TODO: Move latency measurement up after fusion
		// E.g. store LatencyMeasurement in IMUReport until after fusion to add fusion latency ontop
		provider->latencyMeasurements.push_back({ sample.timestamp, { 
			(uint16_t)dtUS(sample.timestamp, sclock::now()) } } );
	}

	LOG(LIO, LTrace, "Tracker %s (%d) got raw IMU sample with %fms latency!",
		tracker->path.c_str(), tracker->id, dtMS(sample.timestamp, sclock::now()));
}

static void handleIMUFused(void *data, const vrpn_IMUFUSEDCB t)
{ // Currently not sent by AsterTrack
	vrpn_Tracker_AsterTrack *tracker = (vrpn_Tracker_AsterTrack*)data;

	if (!tracker->remoteIMU)
		tracker->remoteIMU = registerRemoteIMU(tracker->path);
	if (!tracker->remoteIMU)
	{
		LOG(LIO, LWarn, "Tracker %s (%d) is dropping a fused IMU sample!", tracker->path.c_str(), tracker->id);
		return;
	}

	IMUSampleFused sample;
	sample.timestamp = getTimestamp(t.msg_time);
	sample.accel = Eigen::Vector3d(t.accel[0], t.accel[1], t.accel[2]).cast<float>();
	sample.quat = Eigen::Quaterniond(t.quat[Q_W], t.quat[Q_X], t.quat[Q_Y], t.quat[Q_Z]).cast<float>();

	tracker->remoteIMU->isFused = true;
	tracker->remoteIMU->hasMag = false; // TODO: Technically unknown. Should IMU side advertise capabilities?
	tracker->remoteIMU->samplesFused.push_back(sample);

	auto provider = getRemoteIMUProvider();
	if (provider->recordLatencyMeasurements)
	{ // TODO: Move latency measurement up after fusion
		// E.g. store LatencyMeasurement in IMUReport until after fusion to add fusion latency ontop
		provider->latencyMeasurements.push_back({ sample.timestamp, { 
			(uint16_t)dtUS(sample.timestamp, sclock::now()) } } );
	}

	LOG(LIO, LTrace, "Tracker %s (%d) got fused IMU sample with %fms latency!",
		tracker->path.c_str(), tracker->id, dtMS(sample.timestamp, sclock::now()));
}

vrpn_Tracker_AsterTrack::vrpn_Tracker_AsterTrack(int ID, const char *path, vrpn_Connection *connection, int index)
	: vrpn_Tracker(path, connection),
	vrpn_IMU_Remote(path, vrpn_Tracker::d_connection),
	id(ID), path(path)
{
	vrpn_IMU_Remote::register_change_handler(this, handleIMURaw);
	vrpn_IMU_Remote::register_change_handler(this, handleIMUFused);
}

vrpn_Tracker_AsterTrack::~vrpn_Tracker_AsterTrack()
{
	if (remoteIMU)
		removeRemoteIMU(std::static_pointer_cast<IMUDevice>(remoteIMU));
}

void vrpn_Tracker_AsterTrack::updatePose(int sensor, TimePoint_t time, Eigen::Isometry3f pose)
{
	if (!d_connection->doing_okay())
	{
		LOG(LIO, LDarn, "VRPN Connection for '%s' broken!\n", path.c_str());
		return;
	}
	vrpn_Tracker::timestamp = createTimestamp(time);
	vrpn_Tracker::frame_count = 0; // Unused

	// We're using a left-handed coordinate system (same as Unreal, or a rotated Unity one)
	// VRPN (along with OpenCV, Blender, etc.) use right-handed coordinate systems
	// So flip handedness here
	Eigen::Isometry3f flipXY;
	flipXY.matrix()
	 << 0, 1, 0, 0,
		1, 0, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	pose = flipXY * pose * flipXY;
	// Additionally, the camera points into the positive z direction, which is convenient for CV (z == distance)
	// However, other tools like Blender have the camera point into the negative z direction

	// Set packet data
	d_sensor = sensor;
	Eigen::Quaternionf rotQ(pose.rotation());
	d_quat[Q_X] = rotQ.x();
	d_quat[Q_Y] = rotQ.y();
	d_quat[Q_Z] = rotQ.z();
	d_quat[Q_W] = rotQ.w();
	pos[0] = pose.translation().x();
	pos[1] = pose.translation().y();
	pos[2] = pose.translation().z();

	// Encode and send packet
	static char buffer[1000];
	int length = vrpn_Tracker::encode_to(buffer);
	int error = d_connection->pack_message(length, vrpn_Tracker::timestamp, position_m_id, d_sender_id, buffer, vrpn_CONNECTION_LOW_LATENCY);
	if (error) LOG(LIO, LError, "Failed to write VRPN message with error %d!\n", error);
}

void vrpn_Tracker_AsterTrack::mainloop()
{
	vrpn_Tracker::server_mainloop();

	// While this is a remote object (receiving), we operate it in server mode
	// So the client may send us (the tracking server) IMU samples
	vrpn_IMU_Remote::mainloop_server();
}


/* Wrapper for a VRPN remote tracker */

static void handleTrackerPosRot(void *data, const vrpn_TRACKERCB t)
{
	vrpn_Tracker_Wrapper *tracker = (vrpn_Tracker_Wrapper*)data;
	tracker->lastTimestamp = getTimestamp(t.msg_time);
	tracker->lastPacket = sclock::now();
	tracker->receivedPackets = true;

	if (tracker->logPackets)
	{
		LOG(LIO, LInfo, "Tracker %s, sensor %d pose update!", tracker->path.c_str(), t.sensor);
		LOG(LIO, LInfo, "Pos: (%f, %f, %f)", t.pos[0], t.pos[1], t.pos[2]);
		LOG(LIO, LInfo, "Rot: (%f, %f, %f, %f)", t.quat[0], t.quat[1], t.quat[2], t.quat[3]);
	}

	Eigen::Vector3d pos = Eigen::Vector3d(t.pos[0], t.pos[1], t.pos[2]);
	Eigen::Matrix3d rot = Eigen::Quaterniond(t.quat[Q_W], t.quat[Q_X], t.quat[Q_Y], t.quat[Q_Z]).toRotationMatrix();
	Eigen::Isometry3f pose;
	pose.linear() = rot.cast<float>();
	pose.translation() = pos.cast<float>();
	tracker->pose = pose;
}

static void handleTrackerVelocity(void *data, const vrpn_TRACKERVELCB t)
{ // Currently not sent by AsterTrack
	vrpn_Tracker_Wrapper *tracker = (vrpn_Tracker_Wrapper*)data;
	tracker->lastTimestamp = getTimestamp(t.msg_time);
	tracker->lastPacket = sclock::now();
	tracker->receivedPackets = true;

	if (tracker->logPackets)
	{
		LOG(LIO, LInfo, "Tracker %s, sensor %d velocity update!", tracker->path.c_str(), t.sensor);
		LOG(LIO, LInfo, "dT: (%f, %f, %f)", t.vel[0], t.vel[1], t.vel[2]);
		LOG(LIO, LInfo, "dR: (%f, %f, %f, %f)", t.vel_quat[0], t.vel_quat[1], t.vel_quat[2], t.vel_quat[3]);
	}
}

static void handleTrackerAccel(void *data, const vrpn_TRACKERACCCB t)
{ // Currently not sent by AsterTrack
	vrpn_Tracker_Wrapper *tracker = (vrpn_Tracker_Wrapper*)data;
	tracker->lastTimestamp = getTimestamp(t.msg_time);
	tracker->lastPacket = sclock::now();
	tracker->receivedPackets = true;

	if (tracker->logPackets)
	{
		LOG(LIO, LInfo, "Tracker %s, sensor %d accel update!", tracker->path.c_str(), t.sensor);
		LOG(LIO, LInfo, "ddT: (%f, %f, %f)", t.acc[0], t.acc[1], t.acc[2]);
		LOG(LIO, LInfo, "ddR: (%f, %f, %f, %f)", t.acc_quat[0], t.acc_quat[1], t.acc_quat[2], t.acc_quat[3]);
	}
}

vrpn_Tracker_Wrapper::vrpn_Tracker_Wrapper(std::string Path)
{
	path = std::move(Path);
	if (path.find('@') == path.npos)
		path += "@localhost";
}

void vrpn_Tracker_Wrapper::connect()
{
	remote = std::make_unique<vrpn_Tracker_Remote>(path.c_str());
	remote->register_change_handler(this, handleTrackerPosRot);
	remote->register_change_handler(this, handleTrackerVelocity);
	remote->register_change_handler(this, handleTrackerAccel);
}



std::vector<std::string> vrpn_getKnownTrackers(vrpn_Connection *connection)
{
	std::vector<std::string> trackers;
	// TODO: Implement custom protocol to send trackers to clients?
	// I thought VRPN internally had knowledge on tracker server provides, but that is not the case
	return trackers;
}


/*
 * A horrible hack to allow access to private members of VRPN.
 */

/* #define MOD_CONCATE_(X, Y) X##Y
#define MOD_CONCATE(X, Y) MOD_CONCATE_(X, Y)

#define ALLOW_ACCESS(CLASS, MEMBER, ...) \
  template<typename Only, __VA_ARGS__ CLASS::*Member> \
  struct MOD_CONCATE(MEMBER, __LINE__) { friend __VA_ARGS__ CLASS::*Access(Only*) { return Member; } }; \
  template<typename> struct Only_##MEMBER; \
  template<> struct Only_##MEMBER<CLASS> { friend __VA_ARGS__ CLASS::*Access(Only_##MEMBER<CLASS>*); }; \
  template struct MOD_CONCATE(MEMBER, __LINE__)<Only_##MEMBER<CLASS>, &CLASS::MEMBER>

#define ACCESS(OBJECT, MEMBER) \
 (OBJECT).*Access((Only_##MEMBER<std::remove_reference<decltype(OBJECT)>::type>*)nullptr)
#define ACCESS_AS(OBJECT, TYPE, MEMBER) \
 (OBJECT).*Access((Only_##MEMBER<std::remove_reference<TYPE>::type>*)nullptr)


ALLOW_ACCESS(vrpn_Connection, d_endpoints, vrpn::EndpointContainer);
ALLOW_ACCESS(vrpn_Endpoint, d_dispatcher, vrpn_TypeDispatcher*);

class vrpn_TypeDispatcher {

public:
	int numSenders(void) const;
	const char *senderName(int which) const;
}; */
