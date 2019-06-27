/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#pragma once
#include <stdio.h>
#include <conio.h>
#include <math.h>
#include "kai.h"

class CClientSocket : public kaiSocket
{
public:
	CClientSocket(void);
	virtual ~CClientSocket(void);

	void sendVelocity(float v1, float v2);
	void sendMovePTO(int down);
	void resetPath();
	void resetCoverage();
	void resetPose();
	void reqCoverage();
	void reqSlipAngle();
	void reqGPS();
	void reqIMU();
	void reqVehicleStatus();
	void reqIndicators();

protected:
	virtual void onMessage(kaiMsg &msg);

private:
	kaiMsg *_msgSend;
	double	_curTime;
};
