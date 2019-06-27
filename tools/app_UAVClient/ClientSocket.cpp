/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#include "ClientSocket.h"
#include "UAV_def.h"
#include "UAV_protocol.h"

CClientSocket::CClientSocket(void)
: _msgSend(NULL), _curTime(0)
{
}

CClientSocket::~CClientSocket(void)
{
}

void CClientSocket::sendVelocity(float v1, float v2)
{
	if (!_msgSend) {
		_msgSend = new kaiMsg();
		_msgSend->allocateMemory();
	}
	
	_msgSend->reset();
	_msgSend->id(UAVP_SET_TARGET_VELOCITY);
	_msgSend->begin();
	(*_msgSend) << (float)v1; // target forward velocity or left wheel velocity
	(*_msgSend) << (float)v2; // target steer angle or right wheel velocity
	_msgSend->end();
	send(*_msgSend);
}

void CClientSocket::sendMovePTO(int down)
{
	if (!_msgSend) {
		_msgSend = new kaiMsg();
		_msgSend->allocateMemory();
	}
	
	PTO pto;
	pto.signal = down;

	_msgSend->reset();
	_msgSend->id(UAVP_MOVE_PTO);
	_msgSend->begin();
	_msgSend->writeData(&pto, sizeof(PTO));
	_msgSend->end();
	send(*_msgSend);
}

void CClientSocket::resetPath()
{
	if (!_msgSend) {
		_msgSend = new kaiMsg();
		_msgSend->allocateMemory();
	}

	static float WP[2][5][3] = {
		{
			{   1.0f,   1.0f, 0.0f },
			{  10.0f,  10.0f, 0.0f },
			{  20.0f,  10.0f, 0.0f },
			{  20.0f, -50.0f, 0.0f },
			{ -50.0f, -50.0f, 0.0f }
		},
		{
			{   5.0f,   5.0f, 0.0f },
			{  15.0f,  15.0f, 0.0f },
			{  30.0f,  15.0f, 0.0f },
			{  30.0f, -60.0f, 0.0f },
			{ -60.0f, -60.0f, 0.0f }
		}
	};
	static int c_i = 0;

	_msgSend->reset();
	_msgSend->id(UAVP_ADD_WAYPOINT);
	_msgSend->begin();
	(*_msgSend) << (unsigned int)1 << WP[c_i][0][0] << WP[c_i][0][1] << WP[c_i][0][2];
	_msgSend->end();
	send(*_msgSend);

	_msgSend->reset();
	_msgSend->id(UAVP_ADD_WAYPOINT);
	_msgSend->begin();
	(*_msgSend) << (unsigned int)2 << WP[c_i][1][0] << WP[c_i][1][1] << WP[c_i][1][2];
	_msgSend->end();
	send(*_msgSend);

	_msgSend->reset();
	_msgSend->id(UAVP_ADD_WAYPOINT);
	_msgSend->begin();
	(*_msgSend) << (unsigned int)3 << WP[c_i][2][0] << WP[c_i][2][1] << WP[c_i][2][2];
	_msgSend->end();
	send(*_msgSend);

	_msgSend->reset();
	_msgSend->id(UAVP_ADD_WAYPOINT);
	_msgSend->begin();
	(*_msgSend) << (unsigned int)4 << WP[c_i][3][0] << WP[c_i][3][1] << WP[c_i][3][2];
	_msgSend->end();
	send(*_msgSend);

	_msgSend->reset();
	_msgSend->id(UAVP_ADD_WAYPOINT);
	_msgSend->begin();
	(*_msgSend) << (unsigned int)5 << WP[c_i][4][0] << WP[c_i][4][1] << WP[c_i][4][2];
	_msgSend->end();
	send(*_msgSend);

	c_i = (c_i == 0 ? 1 : 0);
}

void CClientSocket::resetCoverage()
{
	if (!_msgSend) {
		_msgSend = new kaiMsg();
		_msgSend->allocateMemory();
	}

	_msgSend->reset();
	_msgSend->id(UAVP_RESET_COVERAGE);
	_msgSend->begin();
	_msgSend->end();
	send(*_msgSend);
}

void CClientSocket::resetPose()
{
	if (!_msgSend) {
		_msgSend = new kaiMsg();
		_msgSend->allocateMemory();
	}

	_msgSend->reset();
	_msgSend->id(UAVP_SET_VEHICLE_POSE);
	_msgSend->begin();
	(*_msgSend) << (float)10.0f << (float)10.0f << (float)45.0f << (float)0.0f;
	_msgSend->end();
	send(*_msgSend);
}

void CClientSocket::reqCoverage()
{
	if (!_msgSend) {
		_msgSend = new kaiMsg();
		_msgSend->allocateMemory();
	}

	_msgSend->reset();
	_msgSend->id(UAVP_REQ_COVERAGE);
	_msgSend->begin();
	_msgSend->end();
	send(*_msgSend);
}

void CClientSocket::reqSlipAngle()
{
	if (!_msgSend) {
		_msgSend = new kaiMsg();
		_msgSend->allocateMemory();
	}

	_msgSend->reset();
	_msgSend->id(UAVP_REQ_SLIP_ANGLE);
	_msgSend->begin();
	_msgSend->end();
	send(*_msgSend);
}

void CClientSocket::reqGPS()
{
	if (!_msgSend) {
		_msgSend = new kaiMsg();
		_msgSend->allocateMemory();
	}

	_msgSend->reset();
	_msgSend->id(UAVP_REQ_GPS);
	_msgSend->begin();
	_msgSend->end();
	send(*_msgSend);
}

void CClientSocket::reqIMU()
{
	if (!_msgSend) {
		_msgSend = new kaiMsg();
		_msgSend->allocateMemory();
	}

	_msgSend->reset();
	_msgSend->id(UAVP_REQ_IMU);
	_msgSend->begin();
	_msgSend->end();
	send(*_msgSend);
}

void CClientSocket::reqVehicleStatus()
{
	if (!_msgSend) {
		_msgSend = new kaiMsg();
		_msgSend->allocateMemory();
	}

	_msgSend->reset();
	_msgSend->id(UAVP_REQ_VEHICLE_STATUS);
	_msgSend->begin();
	_msgSend->end();
	send(*_msgSend);
}

void CClientSocket::reqIndicators()
{
	if (!_msgSend) {
		_msgSend = new kaiMsg();
		_msgSend->allocateMemory();
	}

	_msgSend->reset();
	_msgSend->id(UAVP_REQ_INDICATORS);
	_msgSend->begin();
	_msgSend->end();
	send(*_msgSend);
}

void CClientSocket::onMessage(kaiMsg &msg)
{
#ifdef _DEBUG
	//printf("msg id:%d, msg size:%d\n", msg.id(), msg.size());
#endif
	switch (msg.id()) {
		case UAVP_ACK_INDICATORS:
			{
				float indicator[4];
				msg.begin();
				msg >> indicator[0] >> indicator[1] >> indicator[2] >> indicator[3];
				printf("UAVP_ACK_INDICATORS: \n\ttractive force(L) = %.3f \n\ttractive force(L) = %.3f \n\tsinkage = %.3f \n\tmotion resistance = %.3f\n",
					indicator[0], indicator[1], indicator[2], indicator[3]);
			}
			break;
		case UAVP_ACK_COVERAGE:
			{
				int coverage[2];
				msg.begin();
				msg >> coverage[1] >> coverage[0];
				printf("UAVP_ACK_COVERAGE: coverage = %d / %d\n", coverage[0], coverage[1]);
			}
			break;
		case UAVP_GET_SENSORS:
			{
				//float rtData[24] = {0,};
				//memcpy(rtData, (char*)msg.buffer() + Size_kaiHEADER, sizeof(float) * 24);
				//puts("Sensor return message.");
			}
			break;
		default:
			break;
	}
}
