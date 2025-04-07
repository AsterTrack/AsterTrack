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

#include "vrpn_imu.hpp"

vrpn_IMU::vrpn_IMU(const char *name, vrpn_Connection *c)
	: vrpn_BaseClass(name, c)
{
	vrpn_BaseClass::init();

	// Set the current time to zero, just to have something there
	timestamp.tv_sec = 0;
	timestamp.tv_usec = 0;

	frame_count = 0;

	// Set the position to the origin and the orientation to identity
	// just to have something there in case nobody fills them in later
	imu_accel[0] = imu_accel[1] = imu_accel[2] = 0.0;
	imu_gyro[0] = imu_gyro[1] = imu_gyro[2] = 0.0;
	imu_mag[0] = imu_mag[1] = imu_mag[2] = 0.0;
	imu_quat[0] = imu_quat[1] = imu_quat[2] = 0.0;
	imu_quat[3] = 1.0;
}

int vrpn_IMU::register_types(void)
{
	// Register this tracker device and the needed message types
	if (d_connection) {
		imu_raw_m_id =
			d_connection->register_message_type("vrpn_IMU Raw");
		imu_fused_m_id =
			d_connection->register_message_type("vrpn_IMU Fused");
		imu_request_id =
			d_connection->register_message_type("vrpn_IMU Request");
	}
	return 0;
}

// virtual
vrpn_IMU::~vrpn_IMU(void)
{
}

// NOTE: you need to be sure that if you are sending vrpn_float64 then
//       the entire array needs to remain aligned to 8 byte boundaries
//	 (malloced data and static arrays are automatically alloced in
//	  this way).  Assumes that there is enough room to store the
//	 entire message.  Returns the number of characters sent.
int vrpn_IMU::encode_imu_raw_to(char *buf)
{
	char *bufptr = buf;
	int buflen = 1000;
	uint32_t reserved = 0;

	// Byte order of each needs to be reversed to match network standard

	// Reserved for future use
	vrpn_buffer(&bufptr, &buflen, reserved);
	vrpn_buffer(&bufptr, &buflen, reserved);

	vrpn_buffer(&bufptr, &buflen, imu_accel[0]);
	vrpn_buffer(&bufptr, &buflen, imu_accel[1]);
	vrpn_buffer(&bufptr, &buflen, imu_accel[2]);

	vrpn_buffer(&bufptr, &buflen, imu_gyro[0]);
	vrpn_buffer(&bufptr, &buflen, imu_gyro[1]);
	vrpn_buffer(&bufptr, &buflen, imu_gyro[2]);

	vrpn_buffer(&bufptr, &buflen, imu_mag[0]);
	vrpn_buffer(&bufptr, &buflen, imu_mag[1]);
	vrpn_buffer(&bufptr, &buflen, imu_mag[2]);

	return 1000 - buflen;
}

int vrpn_IMU::encode_imu_fused_to(char *buf)
{
	char *bufptr = buf;
	int buflen = 1000;
	uint32_t reserved = 0;

	// Byte order of each needs to be reversed to match network standard

	// Reserved for future use
	vrpn_buffer(&bufptr, &buflen, reserved);
	vrpn_buffer(&bufptr, &buflen, reserved);

	vrpn_buffer(&bufptr, &buflen, imu_accel[0]);
	vrpn_buffer(&bufptr, &buflen, imu_accel[1]);
	vrpn_buffer(&bufptr, &buflen, imu_accel[2]);

	vrpn_buffer(&bufptr, &buflen, imu_quat[0]);
	vrpn_buffer(&bufptr, &buflen, imu_quat[1]);
	vrpn_buffer(&bufptr, &buflen, imu_quat[2]);
	vrpn_buffer(&bufptr, &buflen, imu_quat[3]);

	return 1000 - buflen;
}


vrpn_IMU_Local::vrpn_IMU_Local(const char *name, vrpn_Connection *c)
	: vrpn_IMU(name, c)
{
	// Register a handler for the raw change callback from this device.
	if (register_autodeleted_handler(imu_fused_m_id, handle_request_message,
									 this, d_sender_id)) {
		fprintf(stderr,
				"vrpn_IMU_Remote: can't register fused handler\n");
		d_connection = NULL;
	}
	
}

// virtual
vrpn_IMU_Local::~vrpn_IMU_Local(void)
{
}

void vrpn_IMU_Local::mainloop()
{
	server_mainloop();
}

void vrpn_IMU_Local::mainloop_client()
{
	if (d_connection) {
		d_connection->mainloop();
	}
	client_mainloop();
}

int vrpn_IMU_Local::register_change_handler(
	void *userdata, vrpn_IMUREQCHANGEHANDLER handler)
{
	// Ensure that the handler is non-NULL
	if (handler == NULL) {
		fprintf(stderr,
				"vrpn_IMU_Server::register_change_handler: NULL handler\n");
		return -1;
	}
	
	return d_imurequest.register_handler(userdata, handler);
}

int vrpn_IMU_Local::unregister_change_handler(
	void *userdata, vrpn_IMUREQCHANGEHANDLER handler)
{
	return d_imurequest.unregister_handler(userdata, handler);
}

int vrpn_IMU_Local::handle_request_message(void *userdata,
											vrpn_HANDLERPARAM p)
{
	vrpn_IMU_Local *me = (vrpn_IMU_Local *)userdata;
	const char *params = (p.buffer);
	vrpn_IMUREQCB tp;
	int i;

	// Fill in the parameters to the tracker from the message
	if (p.payload_len != (1)) {
		fprintf(stderr, "vrpn_IMU: request message payload error\n");
		fprintf(stderr, "             (got %d, expected %lud)\n", p.payload_len,
				static_cast<unsigned long>(1));
		return -1;
	}
	tp.msg_time = p.msg_time;

	// Reserved for future use
	vrpn_unbuffer(&params, &tp.fused);

	me->d_imurequest.call_handlers(tp);
	return 0;
}

static int VRPN_CALLBACK handle_got_connection(void* data, vrpn_HANDLERPARAM) {
	vrpn_IMU_Remote* d = static_cast<vrpn_IMU_Remote*>(data);
	return d->request_imu_packets();
}

vrpn_IMU_Remote::vrpn_IMU_Remote(const char *name, vrpn_Connection *cn, bool raw, bool fused)
	: vrpn_IMU(name, cn), request_raw(raw), request_fused(fused)
{
	// Make sure that we have a valid connection
	if (d_connection == NULL) {
		fprintf(stderr, "vrpn_IMU_Remote: No connection\n");
		return;
	}

	// Register a handler for the fused change callback from this device.
	if (register_autodeleted_handler(imu_raw_m_id, handle_raw_change_message, this,
									 d_sender_id)) {
		fprintf(stderr,
				"vrpn_IMU_Remote: can't register raw handler\n");
		d_connection = NULL;
	}

	// Register a handler for the raw change callback from this device.
	if (register_autodeleted_handler(imu_fused_m_id, handle_fused_change_message,
									 this, d_sender_id)) {
		fprintf(stderr,
				"vrpn_IMU_Remote: can't register fused handler\n");
		d_connection = NULL;
	}

	// Find out what time it is and put this into the timestamp
	vrpn_gettimeofday(&timestamp, NULL);

	// Create a callback for new connections
	vrpn_int32 got_connection_m_id = d_connection->register_message_type(vrpn_got_connection);
	d_connection->register_handler(got_connection_m_id, handle_got_connection, this);
}

vrpn_IMU_Remote::~vrpn_IMU_Remote()
{
}

int vrpn_IMU_Remote::request_imu_packets()
{
	char msgbuf[2] { request_raw, request_fused };
	if (d_connection->pack_message(2, timestamp, imu_request_id,
									d_sender_id, msgbuf,
									vrpn_CONNECTION_RELIABLE)) {
		fprintf(stderr, "vrpn_IMU_Remote: cannot request IMU samples\n");
		return -1;
	}
	return 0;
}

int vrpn_IMU_Remote::request_imu_packets(bool raw, bool fused)
{
	request_raw = raw;
	request_fused = fused;
	return request_imu_packets();
}

void vrpn_IMU_Remote::mainloop_server()
{
	server_mainloop();
}

void vrpn_IMU_Remote::mainloop()
{
	if (d_connection) {
		d_connection->mainloop();
	}
	client_mainloop();
}

int vrpn_IMU_Remote::register_change_handler(
	void *userdata, vrpn_IMURAWCHANGEHANDLER handler)
{
	// Ensure that the handler is non-NULL
	if (handler == NULL) {
		fprintf(stderr,
				"vrpn_IMU_Remote::register_change_handler: NULL handler\n");
		return -1;
	}
	
	return d_imurawchange.register_handler(userdata, handler);
}

int vrpn_IMU_Remote::unregister_change_handler(
	void *userdata, vrpn_IMURAWCHANGEHANDLER handler)
{
	return d_imurawchange.unregister_handler(userdata, handler);
}

int vrpn_IMU_Remote::register_change_handler(
	void *userdata, vrpn_IMUFUSEDCHANGEHANDLER handler)
{
	// Ensure that the handler is non-NULL
	if (handler == NULL) {
		fprintf(stderr,
				"vrpn_IMU_Remote::register_change_handler: NULL handler\n");
		return -1;
	}
	
	return d_imufusedchange.register_handler(userdata, handler);
}

int vrpn_IMU_Remote::unregister_change_handler(
	void *userdata, vrpn_IMUFUSEDCHANGEHANDLER handler)
{
	return d_imufusedchange.unregister_handler(userdata, handler);
}

int vrpn_IMU_Remote::handle_raw_change_message(void *userdata,
											   vrpn_HANDLERPARAM p)
{
	vrpn_IMU_Remote *me = (vrpn_IMU_Remote *)userdata;
	const char *params = (p.buffer);
	vrpn_int32 padding;
	vrpn_IMURAWCB tp;
	int i;

	// Fill in the parameters to the tracker from the message
	if (p.payload_len != (10 * sizeof(vrpn_float64))) {
		fprintf(stderr, "vrpn_IMU: raw message payload error\n");
		fprintf(stderr, "             (got %d, expected %lud)\n", p.payload_len,
				static_cast<unsigned long>(10 * sizeof(vrpn_float64)));
		return -1;
	}
	tp.msg_time = p.msg_time;

	// Reserved for future use
	vrpn_unbuffer(&params, &padding);
	vrpn_unbuffer(&params, &padding);

	for (i = 0; i < 3; i++) {
		vrpn_unbuffer(&params, &tp.accel[i]);
	}
	for (i = 0; i < 3; i++) {
		vrpn_unbuffer(&params, &tp.gyro[i]);
	}
	for (i = 0; i < 3; i++) {
		vrpn_unbuffer(&params, &tp.mag[i]);
	}

	me->d_imurawchange.call_handlers(tp);
	return 0;
}

int vrpn_IMU_Remote::handle_fused_change_message(void *userdata,
												 vrpn_HANDLERPARAM p)
{
	vrpn_IMU_Remote *me = (vrpn_IMU_Remote *)userdata;
	const char *params = p.buffer;
	vrpn_int32 padding;
	vrpn_IMUFUSEDCB tp;
	int i;

	// Fill in the parameters to the tracker from the message
	if (p.payload_len != (8 * sizeof(vrpn_float64))) {
		fprintf(stderr, "vrpn_IMU: vel message payload error\n");
		fprintf(stderr, "             (got %d, expected %lud)\n", p.payload_len,
				static_cast<unsigned long>(6 * sizeof(vrpn_float64)));
		return -1;
	}
	tp.msg_time = p.msg_time;

	// Reserved for future use
	vrpn_unbuffer(&params, &padding);
	vrpn_unbuffer(&params, &padding);

	for (i = 0; i < 3; i++) {
		vrpn_unbuffer(&params, &tp.accel[i]);
	}
	for (i = 0; i < 4; i++) {
		vrpn_unbuffer(&params, &tp.quat[i]);
	}

	me->d_imufusedchange.call_handlers(tp);
	return 0;
}