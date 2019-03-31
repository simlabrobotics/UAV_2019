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

void CClientSocket::sendVelocity(float lvel_rps, float rvel_rps)
{
	if (!_msgSend) {
		_msgSend = new kaiMsg();
		_msgSend->allocateMemory();
	}
	
	_msgSend->reset();
	_msgSend->id(UAVP_SET_VELOCITY);
	_msgSend->begin();
	(*_msgSend) << (float)lvel_rps; // 
	(*_msgSend) << (float)rvel_rps; // 
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
