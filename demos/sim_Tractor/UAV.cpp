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
#include "UAV.h"
#include "UAV_protocol.h"
#include "UAV_cmd.h"
#include "UAV_conf.h"
extern UAVConf appConf;

UAV::UAV(const string_type& name, const string_type& aml_path, const HTransform& T0, const dVector& q0, bool is_static)
	: rLabServer()
	, _sys(NULL)
	, _drive(NULL)
	, _pto(NULL)
	, _gyro(NULL)
	, _gps(NULL)
	, _acc(NULL)
	, _grid_tool(NULL)
	, _wp(NULL)
	, _wc(NULL)
{
	_name = name;

	_tprintf(_T("UAV> Loading... (%s, %s)\n"), name.c_str(), aml_path.c_str());
	if (is_static)
		_sys = rCreateStaticSystem(aml_path, name, T0, q0);
	else
		_sys = rCreateSystem(aml_path, name, T0, q0);

	if (_sys) 
	{
		_drive     = _sys->findDevice(_T("CarDrive"));
		_pto       = _sys->findDevice(_T("PTO"));
		_gyro      = _sys->findDevice(_T("gyro"));
		_gps       = _sys->findDevice(_T("gps"));
		_acc       = _sys->findDevice(_T("acc"));
		_grid_tool = _sys->findDevice(_T("GRID_TOOL"));
	}
}

UAV::~UAV()
{
}

void UAV::startServer(unsigned short port)
{
	create(port, kaiON_MESSAGE | kaiON_ACCEPT | kaiON_CLOSE, true);
	run(true);
}

void UAV::stopServer()
{
	//stop();
}

void UAV::onClose(int id)
{
	rLabServer::onClose(id);

	if (_drive)
	{
		float val[2];
		val[0] = 0;
		val[1] = 0;
		_drive->writeDeviceValue(val, sizeof(float) * 2, UAVDRV_DATAPORT_SET_VEL_IMMEDIATE);
	}
}

void UAV::onMessage(int id, kaiMsg &msg)
{
	rLabServer::onMessage(id, msg);

	switch (msg.id())
	{
	case UAVP_ADD_WAYPOINT:
	{
		WAYPOINT waypoint;
		memcpy(&waypoint, (char*)msg.buffer() + Size_kaiHEADER, sizeof(waypoint));
		onAddWaypoint(waypoint);
	}
	break;

	case UAVP_SET_TARGET_VELOCITY:
	{
		float val[2] = { 0, 0 };
		memcpy(&val, (char*)msg.buffer() + 8, sizeof(float) * 2);
		onSetTargetVelocity(val[0], val[1]);

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

		if (NULL != _drive)
		{
			float val[6];
			_drive->monitorDeviceValue(val, sizeof(val), UAVDRV_MONITORPORT_POSE);
			float heading = val[3] * RADIAN;
			float x = val[0];
			float y = val[1];
			float z = val[2];
			feedback << heading << x << y << z;
			//printf("Heading angle: %.3f\n", heading);
		}
		else
		{
			float nil(0.0f);
			feedback << nil << nil << nil << nil;
		}

		feedback.id(UAVP_GET_SENSORS);
		feedback.end();
		assert(feedback.size() == 16 && "rLabServer> WARNING! Message(UAVP_GET_SENSORS) size is not 16 bytes.\n");
		client->send(feedback);
	}
	break;

	case UAVP_SET_MAXIMUM_VELOCITY:
	{
		float val[2] = { 0, 0 };
		memcpy(&val, (char*)msg.buffer() + Size_kaiHEADER, sizeof(float) * 2);
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

		if (NULL != _wc)
		{
			int coverage[2];
			_wc->monitorDeviceValue(coverage, sizeof(int) * 2);
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

	case UAVP_RESET_COVERAGE:
	{
		if (NULL != _wc)
		{
			_wc->command(UAVDRV_DATAPORT_RESET_WORK);
		}
	}
	break;

	case UAVP_SET_WORKAREA:
	{
		if (NULL != _wc)
		{
			WORKAREAPOINT pt;
			memcpy(&pt, (char*)msg.buffer() + Size_kaiHEADER, sizeof(WORKAREAPOINT));
			_wc->command(UAVDRV_DATAPORT_SET_WORKAREA, 0, (void*)&pt);
		}
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
			if (0 < _drive->monitorDeviceValue(val, sizeof(val), UAVDRV_MONITORPORT_SLIP_ANGLE))
				feedback << (float)(val[2] * RADIAN) << (float)(val[0]) << (float)(val[1]);
			else
			{
				float nil(0.0f);
				feedback << nil << nil << nil;
			}
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
			_gps->readDeviceValue(val, sizeof(float) * 3);
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

		/*if (NULL != _gyro)
		{
			float val[6];
			_gyro->readDeviceValue(val, sizeof(val));
			float angle[3] = { val[3] * RADIAN, val[4] * RADIAN, val[5] * RADIAN };
			angle[1] *= -1.0f;
			angle[0] += 180.0f;
			if (angle[0] > 180.0f) angle[0] -= 360.0f;
			feedback << angle[2] << angle[1] << angle[0];
		}
		else
		{
			float nil(0.0f);
			feedback << nil << nil << nil;
		}*/

		if (NULL != _drive)
		{
			float val[6];
			_drive->monitorDeviceValue(val, sizeof(val), UAVDRV_MONITORPORT_POSE);
			float heading = val[3] * RADIAN;
			float pitch = val[4] * RADIAN;
			float roll = val[5] * RADIAN;
			feedback << heading << pitch << roll;
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

			if (0 < _drive->monitorDeviceValue(val, sizeof(float) * 2, UAVDRV_MONITORPORT_WHEEL_VELOCITY))
				feedback << val[0] << val[1];
			else
			{
				float nil(0.0f);
				feedback << nil << nil;
			}

			if (0 < _drive->monitorDeviceValue(val, sizeof(float) * 2, UAVDRV_MONITORPORT_VELOCITY_LOCAL))
				feedback << val[0] << val[1];
			else
			{
				float nil(0.0f);
				feedback << nil << nil;
			}
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
			if (0 < _drive->monitorDeviceValue(&param, sizeof(PARAMETER), UAVDRV_MONITORPORT_PARAMETER))
				feedback << param.type << param.param1 << param.param2 << param.param3;
			else
			{
				float nil(0.0f);
				feedback << nil << nil << nil << nil;
			}
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

			if (0 < _drive->monitorDeviceValue(val, sizeof(float) * 2, UAVDRV_MONITORPORT_TRACTIVE_FORCE))
				feedback << val[0] << val[1];
			else
			{
				float nil(0.0f);
				feedback << nil << nil << nil << nil;
			}

			if (0 < _drive->monitorDeviceValue(val, sizeof(float), UAVDRV_MONITORPORT_SINKAGE))
				feedback << val[0];
			else
			{
				float nil(0.0f);
				feedback << nil << nil << nil << nil;
			}

			if (_drive->monitorDeviceValue(val, sizeof(float), UAVDRV_MONITORPORT_MOTION_RESISTANCE))
				feedback << val[0];
			else
			{
				float nil(0.0f);
				feedback << nil << nil << nil << nil;
			}
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

void UAV::onAddWaypoint(const WAYPOINT& waypoint)
{
	// draw path using waypoint device.
	if (NULL != _wp)
	{
		WAYPOINT wp = waypoint;
		_wp->writeDeviceValue(&wp, sizeof(WAYPOINT));
	}
}

void UAV::onSetTargetVelocity(float v1, float v2)
{
	// in case of differential-wheel drive vehicle,
	//		v1: left wheel velocity in rad/sec
	//		v2: right wheel velocity in rad/sec 
	// in case of car-like vehicle,
	//		v1: target forward velocity
	//		v2: target steer angle

	if (NULL == _drive) return;
	float val[2];
	val[0] = v1;
	val[1] = v2;
	_drive->writeDeviceValue(val, sizeof(float) * 2, UAVDRV_DATAPORT_SET_VEL_TARGET);
}

void UAV::onSetMaximumVelocity(float max_v1, float max_v2)
{
	if (NULL == _drive) return;
	float val[2];
	val[0] = max_v1;
	val[1] = max_v2;
	_drive->writeDeviceValue(val, sizeof(float) * 2, UAVDRV_DATAPORT_SET_MAXIMUM_VEL);
}

void UAV::onSetPose(const POSE2D &pose)
{
	if (NULL == _drive) return;
	float val[3];
	val[0] = pose.x;
	val[1] = pose.y;
	val[2] = pose.heading*DEGREE;
	_drive->writeDeviceValue(val, sizeof(float) * 3, UAVDRV_DATAPORT_SET_POSE);
}

void UAV::onMotionState(const MOTIONSTATE &motionstate)
{
	if (NULL == _drive) return;
	float val[2];
	val[0] = motionstate.velocity;
	val[1] = motionstate.heading*DEGREE;
	_drive->writeDeviceValue(val, sizeof(float) * 2, UAVDRV_DATAPORT_SET_VEL_IMMEDIATE);
}

void UAV::onMovingPTO(const PTO &pto)
{
	if (_pto)
	{
		//float angle = (pto.signal == 0 ? 0.0f : 30.0f*DEGREE);
		//_pto->writeDeviceValue(&angle, sizeof(float));
		char active = (char)pto.signal;
		_pto->writeDeviceValue(&active, sizeof(char));
	}
	//if (_wc)
	//{
	//	int activated = (pto.signal == 0 ? 0 : 1);
	//	_wc->writeDeviceValue(&activated, sizeof(int)); // YYY: 작업기가 다 내려간 것을 확인해야 함.
	//}
}

void UAV::onSetParam(const PARAMETER &param)
{
	if (NULL == _drive) return;
	_drive->writeDeviceValue(const_cast<PARAMETER*>(&param), sizeof(PARAMETER), UAVDRV_DATAPORT_SET_PARAMETER);
}

rxDevice* UAV::findDevice(const string_type& name)
{
	rxDevice* device = NULL;
	if (_sys)
		device = _sys->findDevice(name);
	return device;
}

void UAV::datanamesPositionPlot(std::vector<string_type>& datanames, int channel)
{
	datanames.push_back(_name + string_type(_T(".x(m)")));
	datanames.push_back(_name + string_type(_T(".y(m)")));
	datanames.push_back(_name + string_type(_T(".z(m)")));
	datanames.push_back(_name + string_type(_T(".yaw(deg)")));
	datanames.push_back(_name + string_type(_T(".pitch(deg)")));
	datanames.push_back(_name + string_type(_T(".roll(deg)")));
}

void UAV::datanamesVelocityPlot(std::vector<string_type>& datanames, int channel)
{
	datanames.push_back(_name + string_type(_T(".vel_x(m/s)")));
	datanames.push_back(_name + string_type(_T(".vel_x(m/s)")));
	datanames.push_back(_name + string_type(_T(".yaw rate(deg/s)")));
}

void UAV::datanamesGyroPlot(std::vector<string_type>& datanames, int channel)
{
	datanames.push_back(_name + string_type(_T(".yaw(deg)")));
	datanames.push_back(_name + string_type(_T(".pitch(deg)")));
	datanames.push_back(_name + string_type(_T(".roll(deg)")));
}

void UAV::datanamesGPSPlot(std::vector<string_type>& datanames, int channel)
{
	datanames.push_back(_name + string_type(_T(".longitude(m)")));
	datanames.push_back(_name + string_type(_T(".latitude(m)")));
	datanames.push_back(_name + string_type(_T(".altitude(m)")));
}

void UAV::datanamesAccPlot(std::vector<string_type>& datanames, int channel)
{
	datanames.push_back(_name + string_type(_T(".acc_x(m/s^2)")));
	datanames.push_back(_name + string_type(_T(".acc_y(m/s^2)")));
	datanames.push_back(_name + string_type(_T(".acc_z(m/s^2)")));
}

void UAV::datanamesSlipPlot(std::vector<string_type>& datanames, int channel)
{
	datanames.push_back(_name + string_type(_T(".beta slip angle(deg)")));
	datanames.push_back(_name + string_type(_T(".front slip angle(deg)")));
	datanames.push_back(_name + string_type(_T(".rear slip angle(deg)")));
	datanames.push_back(_name + string_type(_T(".forward acceleration by soid deformation(m/s^2)")));
	datanames.push_back(_name + string_type(_T(".lateral acceleration by soid deformation(m/s^2)")));
	datanames.push_back(_name + string_type(_T(".steer angle(deg)")));
	datanames.push_back(_name + string_type(_T(".velocity(m/s)")));
	datanames.push_back(_name + string_type(_T(".lateral velocity(m/s)")));
	datanames.push_back(_name + string_type(_T(".target steer angle(deg)")));
	datanames.push_back(_name + string_type(_T(".target velocity(m/s)")));
}

void UAV::datanamesSlipPlotTracked(std::vector<string_type>& datanames, int channel)
{
	datanames.push_back(_name + string_type(_T(".slip angle(deg)")));
	datanames.push_back(_name + string_type(_T(".slip ratio(left, i_L)")));
	datanames.push_back(_name + string_type(_T(".slip ratio(left, i_R)")));
	datanames.push_back(_name + string_type(_T(".angular velocity(deg/s)")));
	datanames.push_back(_name + string_type(_T(".linear velocity in global x direction(m/s)")));
	datanames.push_back(_name + string_type(_T(".linear velocity in global y direction(m/s)")));
}

void UAV::datanamesLateralForcePlot(std::vector<string_type>& datanames, int channel)
{
	datanames.push_back(_name + string_type(_T(".F_front(N)")));
	datanames.push_back(_name + string_type(_T(".F_rear(N)")));
}

void UAV::collectPositionData(std::vector<double>& data, int channel)
{
	float val[6];

	if (_drive && _drive->monitorDeviceValue(val, sizeof(float) * 6, UAVDRV_MONITORPORT_POSE) > 0)
	{
		data.push_back(val[0]);
		data.push_back(val[1]);
		data.push_back(val[2]);
		data.push_back(val[3] * RADIAN);
		data.push_back(val[4] * RADIAN);
		data.push_back(val[5] * RADIAN);
	}
	else
	{
		float nil(0.0f);
		data.push_back(nil);
		data.push_back(nil);
		data.push_back(nil);
		data.push_back(nil);
		data.push_back(nil);
		data.push_back(nil);
	}
}

void UAV::collectVelocityData(std::vector<double>& data, int channel)
{
	float val[3];

	if (_drive && _drive->monitorDeviceValue(val, sizeof(float) * 3, UAVDRV_MONITORPORT_VELOCITY) > 0)
	{
		data.push_back(val[0]);
		data.push_back(val[1]);
		data.push_back(val[2] * RADIAN);
	}
	else
	{
		float nil(0.0f);
		data.push_back(nil);
		data.push_back(nil);
		data.push_back(nil);
	}
}

void UAV::collectGyroData(std::vector<double>& data, int channel)
{
	float val[6];

	if (_gyro && _gyro->readDeviceValue(val, sizeof(float) * 6) > 0)
	{
		val[4] *= -1.0f;
		val[3] += (float)M_PI;
		if (val[3] > M_PI) val[3] -= (float)(2 * M_PI);
		data.push_back(val[5] * RADIAN);
		data.push_back(val[4] * RADIAN);
		data.push_back(val[3] * RADIAN);
	}
	else
	{
		float nil(0.0f);
		data.push_back(nil);
		data.push_back(nil);
		data.push_back(nil);
	}
}

void UAV::collectGPSData(std::vector<double>& data, int channel)
{
	float val[3];

	if (_gps && _gps->readDeviceValue(val, sizeof(float) * 3) > 0)
	{
		data.push_back(val[0]);
		data.push_back(val[1]);
		data.push_back(val[2]);
	}
	else
	{
		float nil(0.0f);
		data.push_back(nil);
		data.push_back(nil);
		data.push_back(nil);
	}
}

void UAV::collectAccData(std::vector<double>& data, int channel)
{
	float val[3];

	if (_acc && _acc->readDeviceValue(val, sizeof(float) * 3) > 0)
	{
		data.push_back(val[0]);
		data.push_back(val[1]);
		data.push_back(val[2]);
	}
	else
	{
		float nil(0.0f);
		data.push_back(nil);
		data.push_back(nil);
		data.push_back(nil);
	}
}

void UAV::collectSlipData(std::vector<double>& data, int channel)
{
	float val[5];

	if (_drive && _drive->monitorDeviceValue(val, sizeof(float) * 3, UAVDRV_MONITORPORT_SLIP_ANGLE) > 0)
	{
		data.push_back(val[0] * RADIAN); // beta
		data.push_back(val[1] * RADIAN); // beta_front
		data.push_back(val[2] * RADIAN); // beta_rear
	}
	else
	{
		float nil(0.0f);
		data.push_back(nil);
		data.push_back(nil);
		data.push_back(nil);
	}

	if (_drive && _drive->monitorDeviceValue(val, sizeof(float) * 2, UAVDRV_MONITORPORT_SLIP_3D) > 0)
	{
		data.push_back(val[0]);
		data.push_back(val[1]);
	}
	else
	{
		float nil(0.0f);
		data.push_back(nil);
		data.push_back(nil);
	}

	if (_drive && _drive->monitorDeviceValue(val, sizeof(float) * 5, UAVDRV_MONITORPORT_POSE_LOCAL) > 0)
	{
		data.push_back(val[0] * RADIAN); // steer angle
		data.push_back(val[1]);  // current forward(longitudinal) velocity
		data.push_back(val[2]);  // current lateral velocity
		data.push_back(val[3] * RADIAN); // target steer angle
		data.push_back(val[4]);  // target forward velocity
	}
	else
	{
		float nil(0.0f);
		data.push_back(nil);
		data.push_back(nil);
		data.push_back(nil);
	}
}

void UAV::collectSlipDataTracked(std::vector<double>& data, int channel)
{
	float val[3];
	
	if (_drive && _drive->monitorDeviceValue(val, sizeof(float) * 3, UAVDRV_MONITORPORT_SLIP_ANGLE) > 0)
	{
		data.push_back(val[2] * RADIAN);
		data.push_back(val[0]);
		data.push_back(val[1]);
	}
	else
	{
		float nil(0.0f);
		data.push_back(nil);
		data.push_back(nil);
		data.push_back(nil);
	}

	if (_drive && _drive->monitorDeviceValue(val, sizeof(float) * 3, UAVDRV_MONITORPORT_VELOCITY) > 0)
	{
		data.push_back(val[2] * RADIAN); // angular velocity
		data.push_back(val[0]);  // v_x_global
		data.push_back(val[1]);  // v_y_global
	}
	else
	{
		float nil(0.0f);
		data.push_back(nil);
		data.push_back(nil);
		data.push_back(nil);
	}
}

void UAV::collectLateralForceData(std::vector<double>& data, int channel)
{
	float val[2];
	if (_drive && _drive->readDeviceValue(val, sizeof(float) * 2, UAVDRV_MONITORPORT_LATERAL_FORCE) > 0)
	{
		data.push_back(val[0]);
		data.push_back(val[1]);
	}
	else
	{
		float nil(0.0f);
		data.push_back(nil);
		data.push_back(nil);
	}
}
