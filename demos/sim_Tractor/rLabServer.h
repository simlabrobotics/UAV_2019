/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#ifndef __RLABSERVER_H__
#define __RLABSERVER_H__

#include "kai.h"
#include <list>
#include "UAV_def.h"

class rLabServer : public kaiServer
{
public:
	rLabServer();
	virtual void onAccept(int id);
	virtual void onClose(int id);
	virtual void onMessage(int id, kaiMsg &msg);

	void bindSystem(rxSystem* sys, rxEnvironment* env);
	void setTimeStep(double dT);

public:
	virtual void onAddWaypoint(const WAYPOINT &waypoint);
	virtual void onRecvCommand(float lvel_rps, float rvel_rps);
	virtual void onSetMaximumVelocity(float max_vel_rps, float max_acc_rpss);
	virtual void onSetPose(const POSE2D &pose);
	virtual void onMotionState(const MOTIONSTATE &motionstate);
	//virtual void upWorker(const UPWORKER &upworker);
	//virtual void downWorker(const DOWNWORKER &downworker);
	virtual void onMovingPTO(const PTO &pto);
	//virtual void widthofPTO(const PTOWIDTH &widthpto);
	virtual void onSetParam(const PARAMETER &param);

private:
	rxDevice* _drive;
	rxDevice* _gps;
	rxDevice* _gyro;
	rxDevice* _pto; // Power Take Off
	rxDevice* _grid;
	rxDevice* _gridTool;
	double _dT;
	DATAFORMAT _msgSend;
	DATAFORMAT _msgRecv;
	std::list<WAYPOINT> _waypoints;
	int _recv_count;
	int _send_count;
};

#endif // __RLABSERVER_H__
