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

#ifndef VRPN_IMU_H
#define VRPN_IMU_H

#define VRPN_USE_WINSOCK2

#include "vrpn/vrpn_BaseClass.h"

class VRPN_API vrpn_IMU : public vrpn_BaseClass {
public:
	vrpn_IMU(const char *name, vrpn_Connection *c = NULL);

	virtual ~vrpn_IMU(void);

	vrpn_int32 imu_raw_m_id;            // ID of raw IMU message
	vrpn_int32 imu_fused_m_id;          // ID of fused IMU message
	vrpn_int32 imu_request_id;          // ID of IMU request message

	// Description of the next report to go out
	vrpn_float64 imu_accel[3];        // Current IMU acceleration (x,y,z) - might be fused or raw (e.g. including gravity)
	vrpn_float64 imu_gyro[3];         // Current raw IMU gyroscope (x,y,z)
	vrpn_float64 imu_mag[3];          // Current raw IMU magnotometer (x,y,z)
	vrpn_float64 imu_quat[4];         // Current fused IMU quaternion (qx,qy,qz,qw)
	struct timeval timestamp;         // Current timestamp
	vrpn_int32 frame_count;           // Current framecount

	virtual int register_types(void); //< Called by BaseClass init()
	virtual int encode_imu_raw_to(char *buf); // Encodes the IMU report (raw accel&gyro)
	virtual int encode_imu_fused_to(char *buf); // Encodes the IMU report (fused accel&gyro)
};

typedef struct _vrpn_IMUREQCB {
	struct timeval msg_time; // Time of the report
	vrpn_bool fused;         // Request fused samples instead of raw
} vrpn_IMUREQCB;
typedef void(VRPN_CALLBACK *vrpn_IMUREQCHANGEHANDLER)(
	void *userdata, const vrpn_IMUREQCB info);

/**
 * Local IMU Device to transmit to a vrpn_IMU_Remote
 * May operate on the server-side of the connection (normally with mainloop)
 * May also operate on the client-side of the connection (with mainloop_client)
 * This allows a client to connect to a server for absolute tracking but contribute IMU data to the tracking server 
 */
class VRPN_API vrpn_IMU_Local : public vrpn_IMU {
public:
	vrpn_IMU_Local(const char *name, vrpn_Connection *c = NULL);

	virtual ~vrpn_IMU_Local();

	virtual void mainloop();
	virtual void mainloop_client();

	// (un)Register a callback handler to handle a IMU request change
	virtual int register_change_handler(void *userdata,
										vrpn_IMUREQCHANGEHANDLER handler);
	virtual int unregister_change_handler(void *userdata,
										vrpn_IMUREQCHANGEHANDLER handler);

protected:
	vrpn_Callback_List<vrpn_IMUREQCB> d_imurequest;

	static int VRPN_CALLBACK
	handle_request_message(void *userdata, vrpn_HANDLERPARAM p);
};

typedef struct _vrpn_IMURAWCB {
	struct timeval msg_time; // Time of the report
	vrpn_float64 accel[3];   // Raw accelerometer reading of the sensor
	vrpn_float64 gyro[3];    // Raw gyroscope reading of the sensor
	vrpn_float64 mag[3];     // Raw magnotometer reading of the sensor
} vrpn_IMURAWCB;
typedef void(VRPN_CALLBACK *vrpn_IMURAWCHANGEHANDLER)(
	void *userdata, const vrpn_IMURAWCB info);

typedef struct _vrpn_IMUFUSEDCB {
	struct timeval msg_time; // Time of the report
	vrpn_float64 accel[3];   // Fused accelerometer reading of the sensor
	vrpn_float64 quat[4];    // Fused quaternion reading of the sensor
} vrpn_IMUFUSEDCB;
typedef void(VRPN_CALLBACK *vrpn_IMUFUSEDCHANGEHANDLER)(
	void *userdata, const vrpn_IMUFUSEDCB info);

/**
 * Remote IMU Device to receive from a vrpn_IMU_Local
 * May operate on the client-side of the connection (normally with mainloop)
 * May also operate on the server-side of the connection (with mainloop_server)
 * This allows a client to connect to a server for absolute tracking but contribute IMU data to the tracking server 
 */
class VRPN_API vrpn_IMU_Remote : public vrpn_IMU {
public:
	vrpn_IMU_Remote(const char *name, vrpn_Connection *c = NULL, bool raw = true, bool fused = false);

	// unregister all of the handlers registered with the connection
	virtual ~vrpn_IMU_Remote(void);

	int request_imu_packets();
	int request_imu_packets(bool raw, bool fused);

	virtual void mainloop();
	virtual void mainloop_server();

	// (un)Register a callback handler to handle a Raw IMU change
	virtual int register_change_handler(void *userdata,
										vrpn_IMURAWCHANGEHANDLER handler);
	virtual int unregister_change_handler(void *userdata,
										vrpn_IMURAWCHANGEHANDLER handler);

	// (un)Register a callback handler to handle a Fused IMU change
	virtual int register_change_handler(void *userdata,
										vrpn_IMUFUSEDCHANGEHANDLER handler);
	virtual int unregister_change_handler(void *userdata,
										vrpn_IMUFUSEDCHANGEHANDLER handler);

protected:

	bool request_raw;
	bool request_fused;

	vrpn_Callback_List<vrpn_IMURAWCB> d_imurawchange;
	vrpn_Callback_List<vrpn_IMUFUSEDCB> d_imufusedchange;

	static int VRPN_CALLBACK
	handle_raw_change_message(void *userdata, vrpn_HANDLERPARAM p);
	static int VRPN_CALLBACK
	handle_fused_change_message(void *userdata, vrpn_HANDLERPARAM p);
};

#endif // VRPN_IMU_H