/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#ifndef __UAV_H__
#define __UAV_H__

#include <rxSDK/rxSDK.h>
using namespace rlab;
using namespace rlab::rxsdk;

#include <list>
#include "UAV_def.h"
#include "rLabServer.h"

class UAV : public rLabServer
{
public:
	UAV(const string_type& name, const string_type& aml_path, const HTransform& T0, const dVector& q0, bool is_static);
	~UAV();

	void startServer(unsigned short port);
	void stopServer();

public:
	void setDeviceWP(rxDevice* dev) { _wp = dev; }
	void setDeviceWC(rxDevice* dev) { _wc = dev; }
	
	rxDevice* findDevice(const string_type& name);

	// DAQ:
	void datanamesPositionPlot(std::vector<string_type>& datanames, int channel = 0);
	void datanamesVelocityPlot(std::vector<string_type>& datanames, int channel = 0);
	void datanamesGyroPlot(std::vector<string_type>& datanames, int channel = 0);
	void datanamesGPSPlot(std::vector<string_type>& datanames, int channel = 0);
	void datanamesAccPlot(std::vector<string_type>& datanames, int channel = 0);
	void datanamesSlipPlot(std::vector<string_type>& datanames, int channel = 0);
	void datanamesSlipPlotTracked(std::vector<string_type>& datanames, int channel = 0);
	void datanamesLateralForcePlot(std::vector<string_type>& datanames, int channel = 0);
	void collectPositionData(std::vector<double>& data, int channel = 0);
	void collectVelocityData(std::vector<double>& data, int channel = 0);
	void collectGyroData(std::vector<double>& data, int channel = 0);
	void collectGPSData(std::vector<double>& data, int channel = 0);
	void collectAccData(std::vector<double>& data, int channel = 0);
	void collectSlipData(std::vector<double>& data, int channel = 0);
	void collectSlipDataTracked(std::vector<double>& data, int channel = 0);
	void collectLateralForceData(std::vector<double>& data, int channel = 0);

public:
	// override rLabServer:
	virtual void onClose(int id);
	virtual void onMessage(int id, kaiMsg &msg);

protected:
	// network message handlers:
	void onAddWaypoint(const WAYPOINT &waypoint);
	void onSetTargetWheelVelocity(float lvel_rps, float rvel_rps);
	void onSetTargetVelocity(float v_mps, float w_rps);
	void onSetMaximumWheelVelocity(float max_vel_rps, float max_acc_rpss);
	void onSetMaximumVelocity(float max_v_mps, float max_w_rps);
	void onSetPose(const POSE2D &pose);
	void onMotionState(const MOTIONSTATE &motionstate);
	void onMovingPTO(const PTO &pto);
	void onSetParam(const PARAMETER &param);

private:
	string_type _name;
	rxSystem* _sys;
	rxDevice* _drive;
	rxDevice* _pto;			// Power Take Off
	rxDevice* _grid_tool;
	rxDevice* _wp;			// Waypoint Renderer
	rxDevice* _wc;			// Work Area Calculator/Renderer
	rxDevice* _gyro;
	rxDevice* _gps;
	rxDevice* _acc;

	DATAFORMAT _msgSend;
	DATAFORMAT _msgRecv;
	std::list<WAYPOINT> _waypoints;
};

#endif // __UAV_H__
