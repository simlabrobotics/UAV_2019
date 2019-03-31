/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#include "stdafx.h"

#include <rlab/math/rMath.h>
#include <rlab/command/rCmdDefine.h>
#include <rlab/utils/rPerformanceProbe.h>
#include <rlab/utils/rCustomDraw.h>
#include <rxSDK/rxSDK.h>
using namespace rlab;
using namespace rlab::rxsdk;
#include "rLabServer.h"
#include "UAV_protocol.h"
#include "../../devices/rDeviceCombineDrive/rDeviceCombineDriveCmd.h"
#include "../common/HeightMap.h"
extern HeightMap hmap;

bool up   = false;
bool down = false;

static void dumpMessage(kaiMsg& msg, const char* dec)
{
	float arg[4];
	memcpy(arg, (char*)msg.buffer() + 8, sizeof(float)*4);
	printf("%s[id=%02d] %.3f\t %.3f\t %.3f\t %.3f\n",
		dec, msg.id(), arg[0], arg[1], arg[2], arg[3]);
}

rLabServer::rLabServer()
: kaiServer()
, _drive(NULL)
, _gps(NULL)
, _gyro(NULL)
, _pto(NULL)
, _grid(NULL)
, _gridTool(NULL)
, _dT(0)
, _recv_count(0)
, _send_count(0)
{
}

void rLabServer::onMessage(int id, kaiMsg &msg) 
{
	if ((++_recv_count)%1000 == 0)
	{
		printf(">> rLabServer: %d commands received.\n", _recv_count);
	}

	dumpMessage(msg, "<< ");

	switch(msg.id()) 
	{
	case UAVP_ADD_WAYPOINT:
		{
			WAYPOINT waypoint;
			memcpy(&waypoint, (char*)msg.buffer() + Size_kaiHEADER, sizeof(waypoint));
			onAddWaypoint(waypoint);
		}
		break;

	case UAVP_SET_VELOCITY:
		{
			float val[2] = {0, 0};
			memcpy(&val, (char*)msg.buffer() + 8, sizeof(float)*2); // left & right wheel velocity in rad/sec 
			onRecvCommand(val[0], val[1]);

			/*PHYSICS_WORLD->activateWorld();
			if (PHYSICS_WORLD->time() > 0) 
			{
			}
			else 
			{
				PHYSICS_WORLD->update();
			}
			PHYSICS_WORLD->update();
			PHYSICS_WORLD->deactivateWorld();*/

			kaiSocket* client = findClient(id);
			if (!client) break;
			kaiMsg feedback;
			feedback.allocateMemory();
			feedback.reset();
			feedback.begin();

			if (NULL != _gyro)
			{
				float val[6];
				_gyro->readDeviceValue(val, sizeof(val));
				float angle = val[5]*RADIAN;
				feedback << angle;
				//printf("Heading angle: %.3f\n", angle);
			}
			else
			{
				float nil(0.0f);
				feedback << nil;
			}

			if (NULL != _gps)
			{
				float val[3];
				_gps->readDeviceValue(val, sizeof(float)*3);
				feedback << val[0] << val[1] << val[2];
				_send_count++;
				//printf("<< GPS(%d): %.1f\t%.1f\t%.1f \n",  _send_count, val[0], val[1], val[2]);
			}
			else
			{
				float nil(0.0f);
				feedback << nil << nil << nil; 
			}

			feedback.id(UAVP_GET_SENSORS);
			feedback.end();
			assert(feedback.size() == 16 && "rLabServer> WARNING! Message(UAVP_GET_SENSORS) size is not 16 bytes.\n");
			client->send(feedback);
		}
		break;

	case UAVP_SET_MAXIMUM_VELOCITY:
		{
			float val[2] = {0, 0};
			memcpy(&val, (char*)msg.buffer() + Size_kaiHEADER, sizeof(float)*2); // left & right wheel maximum velocity in rad/sec 
			onSetMaximumVelocity(val[0], val[1]);
		}
		break;

	case UAVP_MOVE_PTO:
		{
			PTO pto;
			memcpy(&pto, (char*)msg.buffer() + Size_kaiHEADER, sizeof(PTO));
			onMovingPTO(pto);
		}
		break;

	case UAVP_SET_VEHICLE_POSE:
		{
			POSE2D pose;
			memcpy(&pose, (char*)msg.buffer() + Size_kaiHEADER, sizeof(POSE2D));
			onSetPose(pose);
		}
		break;

	case UAVP_MOTION_STATE:
		{
			MOTIONSTATE motionstate;
			memcpy(&motionstate, (char*)msg.buffer() + Size_kaiHEADER, sizeof(MOTIONSTATE));
			onMotionState(motionstate);
		}
		break;

	/*case UAVP_UP_WORKER:
		{
			UPWORKER upworker;
			memcpy(&upworker, (char*)msg.buffer() + Size_kaiHEADER, sizeof(float));
			upWorker(upworker);
		}
		break;*/

	/*case UAVP_DOWN_WORKER:
		{
			DOWNWORKER downworker;
			memcpy(&downworker, (char*)msg.buffer() + Size_kaiHEADER, sizeof(float));
			downWorker(downworker);
		}
		break;*/

	/*case UAVP_PTO_WIDTH:
		{
			PTOWIDTH widthpto;
			memcpy(&widthpto, (char*)msg.buffer() + Size_kaiHEADER, sizeof(float));
			widthofPTO(widthpto);
		}
		break;*/

	case UAVP_REQ_COVERAGE:
		{
			kaiSocket* client = findClient(id);
			if (!client) break;
			kaiMsg feedback;
			feedback.allocateMemory();
			feedback.reset();
			feedback.begin();

			if (NULL != _grid)
			{
				int coverage[2];
				_grid->monitorDeviceValue(coverage, sizeof(int)*2);
				feedback << coverage[0] << coverage[1];
			}
			else
			{
				int nil(0);
				feedback << nil << nil;
			}

			float reserved(0.0f);
			feedback << reserved << reserved;

			feedback.id(UAVP_ACK_COVERAGE);
			feedback.end();
			assert(feedback.size() == 16 && "rLabServer> WARNING! Message(UAVP_ACK_COVERAGE) size is not 16 bytes.\n");
			client->send(feedback);
		}
		break;

	case UAVP_REQ_SLIP_ANGLE:
		{
			kaiSocket* client = findClient(id);
			if (!client) break;
			kaiMsg feedback;
			feedback.allocateMemory();
			feedback.reset();
			feedback.begin();

			if (NULL != _drive)
			{
				float val[3];
				_drive->monitorDeviceValue(val, sizeof(val), COMBINEDRIVE_MONITORPORT_SLIP_ANGLE);
				feedback << (float)(val[2]*RADIAN) << (float)(val[0]) << (float)(val[1]);
			}
			else
			{
				float nil(0.0f);
				feedback << nil << nil << nil;
			}

			float reserved(0.0f);
			feedback << reserved;

			feedback.id(UAVP_ACK_SLIP_ANGLE);
			feedback.end();
			assert(feedback.size() == 16 && "rLabServer> WARNING! Message(UAVP_ACK_SLIP_ANGLE) size is not 16 bytes.\n");
			client->send(feedback);
		}
		break;

	case UAVP_REQ_GPS:
		{
			kaiSocket* client = findClient(id);
			if (!client) break;
			kaiMsg feedback;
			feedback.allocateMemory();
			feedback.reset();
			feedback.begin();

			if (NULL != _gps)
			{
				float val[3];
				_gps->readDeviceValue(val, sizeof(float)*3);
				feedback << val[0] << val[1] << val[2];
			}
			else
			{
				float nil(0.0f);
				feedback << nil << nil << nil; 
			}

			float reserved(0.0f);
			feedback << reserved;

			feedback.id(UAVP_ACK_GPS);
			feedback.end();
			assert(feedback.size() == 16 && "rLabServer> WARNING! Message(UAVP_ACK_GPS) size is not 16 bytes.\n");
			client->send(feedback);
		}
		break;

	case UAVP_REQ_IMU:
		{
			kaiSocket* client = findClient(id);
			if (!client) break;
			kaiMsg feedback;
			feedback.allocateMemory();
			feedback.reset();
			feedback.begin();

			if (NULL != _gyro)
			{
				float val[6];
				_gyro->readDeviceValue(val, sizeof(val));
				float angle[3] = {val[3]*RADIAN, val[4]*RADIAN, val[5]*RADIAN};
				angle[1] *= -1.0f;
				angle[0] += 180.0f;
				if (angle[0] > 180.0f) angle[0] -= 360.0f;
				feedback << angle[2] << angle[1] << angle[0];
			}
			else
			{
				float nil(0.0f);
				feedback << nil << nil << nil;
			}

			float reserved(0.0f);
			feedback << reserved;

			feedback.id(UAVP_ACK_IMU);
			feedback.end();
			assert(feedback.size() == 16 && "rLabServer> WARNING! Message(UAVP_ACK_IMU) size is not 16 bytes.\n");
			client->send(feedback);
		}
		break;

	case UAVP_REQ_VEHICLE_STATUS:
		{
			kaiSocket* client = findClient(id);
			if (!client) break;
			kaiMsg feedback;
			feedback.allocateMemory();
			feedback.reset();
			feedback.begin();

			if (NULL != _drive)
			{
				float val[2];

				_drive->monitorDeviceValue(val, sizeof(float)*2, COMBINEDRIVE_MONITORPORT_WHEEL_VELOCITY);
				feedback << val[0] << val[1];

				_drive->monitorDeviceValue(val, sizeof(float)*2, COMBINEDRIVE_MONITORPORT_VELOCITY_LOCAL);
				feedback << val[0] << val[1];
			}
			else
			{
				float nil(0.0f);
				feedback << nil << nil << nil << nil;
			}

			feedback.id(UAVP_ACK_VEHICLE_STATUS);
			feedback.end();
			assert(feedback.size() == 16 && "rLabServer> WARNING! Message(UAVP_ACK_VEHICLE_STATUS) size is not 16 bytes.\n");
			client->send(feedback);
		}
		break;

	case UAVP_SET_PARAMETER:
		{
			PARAMETER param;
			memcpy(&param, (char*)msg.buffer() + Size_kaiHEADER, sizeof(PARAMETER));
			onSetParam(param);
		}
		break;

	case UAVP_REQ_PARAMETER:
		{
			PARAMETER param;
			memcpy(&param, (char*)msg.buffer() + Size_kaiHEADER, sizeof(PARAMETER));

			kaiSocket* client = findClient(id);
			if (!client) break;
			kaiMsg feedback;
			feedback.allocateMemory();
			feedback.reset();
			feedback.begin();

			if (NULL != _drive)
			{
				_drive->monitorDeviceValue(&param, sizeof(PARAMETER), COMBINEDRIVE_MONITORPORT_PARAMETER);
				feedback << param.type << param.param1 << param.param2 << param.param3;
			}
			else
			{
				float nil(0.0f);
				feedback << nil << nil << nil << nil;
			}

			feedback.id(UAVP_ACK_PARAMETER);
			feedback.end();
			assert(feedback.size() == 16 && "rLabServer> WARNING! Message(UAVP_ACK_PARAMETER) size is not 16 bytes.\n");
			client->send(feedback);
		}
		break;

	case UAVP_REQ_INDICATORS:
		{
			kaiSocket* client = findClient(id);
			if (!client) break;
			kaiMsg feedback;
			feedback.allocateMemory();
			feedback.reset();
			feedback.begin();

			if (NULL != _drive)
			{
				float val[2];

				_drive->monitorDeviceValue(val, sizeof(float)*2, COMBINEDRIVE_MONITORPORT_TRACTIVE_FORCE);
				feedback << val[0] << val[1];

				_drive->monitorDeviceValue(val, sizeof(float), COMBINEDRIVE_MONITORPORT_SINKAGE);
				feedback << val[0];

				_drive->monitorDeviceValue(val, sizeof(float), COMBINEDRIVE_MONITORPORT_MOTION_RESISTANCE);
				feedback << val[0];
			}
			else
			{
				float nil(0.0f);
				feedback << nil << nil << nil << nil;
			}

			feedback.id(UAVP_ACK_INDICATORS);
			feedback.end();
			assert(feedback.size() == 16 && "rLabServer> WARNING! Message(UAVP_ACK_INDICATORS) size is not 16 bytes.\n");
			client->send(feedback);
		}
		break;
	}
}

void rLabServer::bindSystem(rxSystem* sys, rxEnvironment* env)
{
	if (sys)
	{
		_drive = sys->findDevice(_T("CarDrive"));
		_gps = sys->findDevice(_T("gps"));
		_gyro = sys->findDevice(_T("gyro"));
		_pto = sys->findDevice(_T("PTO"));
		//_grid = sys->findDevice(_T("GRID"));
		_gridTool = sys->findDevice(_T("GRID_TOOL"));
	}
	if (env)
	{
		rxSystem* workAreaCalc = env->findSystem(_T("WorkAreaCalc.aml"));
		if (workAreaCalc) {
			_grid = workAreaCalc->findDevice(_T("GRID"));
		}
	}
}

void rLabServer::setTimeStep(double dT)
{
	_dT = dT;
}

void rLabServer::onAddWaypoint(const WAYPOINT& waypoint)
{
	char wp_name[16] = {0, };
	sprintf(wp_name, "wp_%d", waypoint.index);

	//std::vector<rMath::Vector3D> vertexPoints;

	if (waypoint.index == 1/*&& 첫번째 웨이 포인트인 경우*/)
	{
		_waypoints.clear();

		if (NULL != _drive )
		{
			float val[3];
			val[0] = waypoint.x;
			val[1] = waypoint.y;
			val[2] = waypoint.angle*DEGREE;
			///val[2] = 0;
			_drive->writeDeviceValue(val ,sizeof(float)*3, COMBINEDRIVE_DATAPORT_SET_POSE);
		}
	}

	WAYPOINT wp0;
	if (_waypoints.empty())
		wp0 = waypoint;
	else
		wp0 = _waypoints.front();

	_waypoints.push_front(waypoint);

	// draw waypoint
	float height, height0;
	hmap.GetHeight(wp0.x, wp0.y, height0);
	hmap.GetHeight(waypoint.x, waypoint.y, height);
	rID cid = INVALID_RID;
	rlab::utils::rCustomDrawInfo drawInfo;
	drawInfo.name = wp_name;
	drawInfo.flag = rlab::utils::CUSTOM_DRAW_NEW;
	drawInfo.drawType = rlab::utils::CUSTOM_DRAW_POLYLINE;
	drawInfo.lineWidth = 5;
	drawInfo.T.translation()[0] = waypoint.x;
	drawInfo.T.translation()[1] = waypoint.y;
	drawInfo.T.translation()[2] = height+1.0;
	drawInfo.color[0] = 1.0;
	drawInfo.color[1] = 0;
	drawInfo.color[2] = 0;
	drawInfo.color[3] = 0.4;
	drawInfo.vertexCnt = 2;
	drawInfo.vertexPoints.push_back(Vector3D(0, 0, -0.5));
	drawInfo.vertexPoints.push_back(Vector3D(wp0.x-waypoint.x, wp0.y-waypoint.y, height0-height-0.5));
	drawInfo.vertexNormals.push_back(Vector3D(0.0, 0.0, 1.0));
	drawInfo.vertexNormals.push_back(Vector3D(0.0, 0.0, 1.0));
	drawInfo.vertexColors.push_back(rColor(1.0, 0.0, 0.0, 0.4));
	drawInfo.vertexColors.push_back(rColor(1.0, 0.0, 0.0, 0.4));

	cid = PHYSICS_WORLD.customDraw(drawInfo);
}

void rLabServer::onRecvCommand(float lvel_rps, float rvel_rps)
{
	if (NULL == _drive) return;
	float val[2];
	val[0] = lvel_rps;
	val[1] = rvel_rps;
	_drive->writeDeviceValue(val ,sizeof(float)*2);
}

void rLabServer::onSetMaximumVelocity(float max_vel_rps, float max_acc_rpss)
{
	if (NULL == _drive) return;
	float val[2];
	val[0] = max_vel_rps;
	val[1] = max_acc_rpss;
	_drive->writeDeviceValue(val ,sizeof(float)*2);
}

void rLabServer::onSetPose(const POSE2D &pose)
{
	if (NULL == _drive) return;
	float val[3];
	val[0] = pose.x;
	val[1] = pose.y;
	val[2] = pose.heading*DEGREE;
	_drive->writeDeviceValue(val ,sizeof(float)*3, COMBINEDRIVE_DATAPORT_SET_POSE);
}

void rLabServer::onMotionState(const MOTIONSTATE &motionstate)
{
	if (NULL == _drive) return;
	float val[2];
	val[0] = motionstate.velocity;
	val[1] = motionstate.heading*DEGREE;
	_drive->writeDeviceValue(val, sizeof(float)*2, COMBINEDRIVE_DATAPORT_SET_VEL_IMMEDIATE);
}

//void rLabServer::upWorker(const UPWORKER &upworker)
//{
//	if (NULL == _pto) return;
//	float val = upworker.up*DEGREE;
//	_pto->writeDeviceValue(&val, sizeof(float));
//}

//void rLabServer::downWorker(const DOWNWORKER &downworker)
//{
//	if (NULL == _pto) return;
//	float val = -downworker.down*DEGREE;
//	_pto->writeDeviceValue(&val, sizeof(float));
//}

void rLabServer::onMovingPTO(const PTO &pto)
{
	if (_pto) 
	{
		float angle = (pto.signal == 0 ? 0.0f : -15.0f*DEGREE);
		_pto->writeDeviceValue(&angle, sizeof(float));
	}
	if (_grid) 
	{
		int activated = (pto.signal == 0 ? 0 : 1);
		_grid->writeDeviceValue(&activated, sizeof(int));
	}
}

void rLabServer::onSetParam(const PARAMETER &param)
{
	if (NULL == _drive) return;
	_drive->writeDeviceValue(const_cast<PARAMETER*>(&param), sizeof(PARAMETER), COMBINEDRIVE_DATAPORT_SET_PARAMETER);
}

//void rLabServer::widthofPTO(const PTOWIDTH &widthpto)
//{
//	/*if (NULL == _gridTool) return;
//	float val = widthpto.width;
//	_gridTool->writeDeviceValue(&val, sizeof(float));*/
//}

void rLabServer::onAccept(int id)
{
	kaiSocket* client = findClient(id);
	if (client)
	{
		printf(">> New controller(%d) is connected.\n", id);
	}
}

void rLabServer::onClose(int id)
{
	kaiSocket* client = findClient(id);
	if (client)
	{
		printf(">> Controller(%d) is disconnected.\n", id);

		if (_drive)
		{
			float val[2];
			val[0] = 0;
			val[1] = 0;
			_drive->writeDeviceValue(val, sizeof(float)*2, COMBINEDRIVE_DATAPORT_SET_VEL_IMMEDIATE);
		}
	}
}
