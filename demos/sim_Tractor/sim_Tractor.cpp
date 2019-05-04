/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */

//--------------------------------------------------------------------------------
// UAV (Unmanned Agricultural Vehicle) simulator
//--------------------------------------------------------------------------------

#include "stdafx.h"

#include <rlab/math/rMath.h>
#include <rlab/utils/rCodeUtil.h>
#include <rlab/utils/rPerformanceProbe.h>
#include <rxSDK/rxSDK.h>
using namespace rlab;
using namespace rlab::rxsdk;
#include "rLabServer.h"
#include "UAV_conf.h"
#include "UAV_cmd.h"

UAVConf appConf;
rLabServer svr;

bool bContact = true;		// Enables/disables contact dynamics.
bool bQuit = false;			// Set this flag true to quit this program.
bool bRun = false;			// Set this flag true to activate the program.
										// See OnKeyRun() function for the details.
rTime delT = 0.001;
unsigned int listenPort = 5150;
unsigned int controlPort[MAX_UAV_NUM];
string_type aml_path[MAX_UAV_NUM];
string_type aml_name[MAX_UAV_NUM];
HTransform aml_T0[MAX_UAV_NUM];
dVector aml_q0[MAX_UAV_NUM];
rxSystem* sys[MAX_UAV_NUM];
string_type eml_path = _T("models/Environment/Ground/ground.eml");
string_type eml_name = _T("Environment");
HTransform eml_T0;
rxEnvironment* env = NULL;
bool bEnvironment = false;

void ParseCommand(int argc, _TCHAR* argv[]);
void PrintUsage(int argc, _TCHAR* argv[]);
void PrintInstruction();
void SetupDAQ();
void MyKeyboardHandler(int key, void* data);
void MyControlCallback(rTime time, void* data);
void GPSDataCallback(std::vector<double>& data);
void GyroDataCallback(std::vector<double>& data);
void AccDataCallback(std::vector<double>& data);
void VehicleDataCallback(std::vector<double>& data);
void SlipDataCallback(std::vector<double>& data);
void SlipDataCallbackTracked(std::vector<double>& data);
void LateralForceDataCallback(std::vector<double>& data);

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
float v_des = 0.0f; // desired velocity
float theta_des = 0.0f; // desired steer angle
const float V_INC = 0.27f;//0.01;
const float THETA_INC = (float)(5.0*DEGREE);
float pto_des = 0.0f;
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

int _tmain(int argc, _TCHAR* argv[])
{
	ParseCommand(argc, argv);

	rSetLogLevel(255);
	rCreateWorld(bContact, delT, rTimer_MODE_REAL_WIN32);
	rSetGravity(0, 0, -(float)GRAV_ACC);
	rCreatePlane(0, 0, 1, 0);

	for (int uav_no = 0; uav_no < MAX_UAV_NUM; uav_no++)
	{
		if (appConf.enableAml(uav_no))
		{
			_tprintf(_T("Loading... (%s)\n"), aml_path[uav_no].c_str());
			if (appConf.amlIsStatic(uav_no))
				sys[uav_no] = rCreateStaticSystem(aml_path[uav_no], aml_name[uav_no], aml_T0[uav_no], aml_q0[uav_no]);
			else
				sys[uav_no] = rCreateSystem(aml_path[uav_no], aml_name[uav_no], aml_T0[uav_no], aml_q0[uav_no]);
	}

	if (bEnvironment)
		env = rCreateEnvironment(eml_path, eml_name, eml_T0);

	svr.bindSystem(sys, env);

	rInitializeEx(true, true);

	rSetKeyboardMode(rKEYBOARD_MODE_NETWORK);
	rAddKeyboardHandler(MyKeyboardHandler, NULL);

	if (appConf.enableDebugControlLocal()) 
	{
		rAddControlHandler(MyControlCallback, NULL);
	}
	else 
	{
		// initialize rLabServer
		//svr.bindSystem(sys, env);
		svr.setTimeStep(delT);
		svr.create(controlPort, kaiON_MESSAGE | kaiON_ACCEPT | kaiON_CLOSE, true);
		svr.run(true);
	}

	SetupDAQ();
	PrintInstruction();

	rActivateWorld();
	while (!bQuit)
	{
		rUpdate();
		Sleep(10);
	}

	if (!appConf.enableDebugControlLocal()) 
	{
		svr.stop();
	}
	DESTROY_PHYSICS_WORLD();
	DESTROY_DATA_ACQUISITION();

	return 0;
}

void MyKeyboardHandler(int key, void* data)
{
	switch (key)
	{
	case VK_UP: v_des += V_INC; break;
	case VK_DOWN: v_des -= V_INC; break;
	case VK_LEFT: theta_des += THETA_INC; break;
	case VK_RIGHT: theta_des -= THETA_INC; break;
	case VK_SPACE: v_des = 0; break;

	case VK_PRIOR: pto_des = (float)(0 * DEGREE); break;
	case VK_NEXT: pto_des = (float)(30 * DEGREE); break;

	case VK_T:
		rShowTrace(aml_name, _T("Body"), BLUE);
		break;

	case VK_W: // way-point demo
		{
			if (appConf.enableDebugControlLocal()) 
			{
				WAYPOINT wp;
				for (int i=0; i<=10; i++)
				{
					wp.index = i+1; // 1-base index
					wp.x = -10.0f + (float)i * 2.0f;
					wp.y = -10.0f + (float)i * 2.0f;
					wp.angle = 45.0f;
					svr.onAddWaypoint(wp);
				}
			}
		}
		break;

	case VK_TAB:
		{
			/*bRun = !bRun;
			if(bRun)
				rActivateWorld();
			else
				rDeactivateWorld();*/
		}
		break;
	
	case VK_Q:
		{
			rDeactivateWorld();
			bQuit = true;
			//rQuit();
		}
		break;
	}
}

void MyControlCallback(rTime time, void* data)
{
	if (!sys) return;

	rxDevice* carDrive = sys->findDevice(_T("CarDrive"));
	if (carDrive)
	{
		float val[2];
		val[0] = v_des;
		val[1] = theta_des;
		carDrive->writeDeviceValue(val, sizeof(val), UAVDRV_DATAPORT_SET_VEL_TARGET);
	}

	rxDevice* pto = sys->findDevice(_T("PTO"));
	if (pto)
	{
		float val;
		val = pto_des;
		pto->writeDeviceValue(&val, sizeof(val));
	}

	if (env && env->findSystem(_T("Environment::WorkAreaCalc.aml")))
	{
		rxDevice* grid = env->findSystem(_T("Environment::WorkAreaCalc.aml"))->findDevice(_T("GRID"));
		if (grid)
		{
			//int cmd = (pto_des < 0 ? 1 : 0);
			int cmd = pto_des == 0 ? 0 : 1;
			grid->writeDeviceValue(&cmd, sizeof(int));
		}
	}
}

void GPSDataCallback(std::vector<double>& data)
{
	if (!sys) return;
	rxDevice* gps = sys->findDevice(_T("gps"));
	if (!gps) return;
	float val[3];
	gps->readDeviceValue(val, sizeof(val));
	data.push_back(val[0]);
	data.push_back(val[1]);
	data.push_back(val[2]);
}

void GyroDataCallback(std::vector<double>& data)
{
	if (!sys) return;
	rxDevice* gyro = sys->findDevice(_T("gyro"));
	if (!gyro) return;
	float val[6];
	gyro->readDeviceValue(val, sizeof(val));
	val[4] *= -1.0f;
	val[3] += M_PI;
	if (val[3] > M_PI) val[3] -= 2*M_PI;
	data.push_back(val[5]*RADIAN);
	data.push_back(val[4]*RADIAN);
	data.push_back(val[3]*RADIAN);
}

void AccDataCallback(std::vector<double>& data)
{
	if (!sys) return;
	rxDevice* acc = sys->findDevice(_T("acc"));
	if (!acc) return;
	float val[3];
	acc->readDeviceValue(val, sizeof(val));
	data.push_back(val[0]);
	data.push_back(val[1]);
	data.push_back(val[2]);
}

void VehicleDataCallback(std::vector<double>& data)
{
	if (!sys) return;
	rxDevice* carDrive = sys->findDevice(_T("CarDrive"));
	if (!carDrive) return;
	float val[5];

	if (appConf.enablePlotPosition()) {
		if (0 < carDrive->monitorDeviceValue(val, sizeof(val), UAVDRV_MONITORPORT_POSE))
		{
			data.push_back(val[0]);
			data.push_back(val[1]);
			data.push_back(val[2] * RADIAN);
			data.push_back(val[3] * RADIAN);
			data.push_back(val[4] * RADIAN);
		}
		else
		{
			float nil(0.0f);
			data.push_back(nil);
			data.push_back(nil);
			data.push_back(nil);
			data.push_back(nil);
			data.push_back(nil);
		}
	}

	if (appConf.enablePlotVelocity()) {
		if (0 < carDrive->monitorDeviceValue(val, sizeof(val), UAVDRV_MONITORPORT_VELOCITY))
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
}

void SlipDataCallback(std::vector<double>& data)
{
	if (!sys) return;
	rxDevice* carDrive = sys->findDevice(_T("CarDrive"));
	if (!carDrive) return;
	float val[3];
	if (0 < carDrive->monitorDeviceValue(val, sizeof(val), UAVDRV_MONITORPORT_SLIP_ANGLE))
	{
		data.push_back(val[0] * RADIAN); // beta
		data.push_back(val[1] * RADIAN);
		data.push_back(val[2] * RADIAN);
	}
	else
	{
		float nil(0.0f);
		data.push_back(nil);
		data.push_back(nil);
		data.push_back(nil);
	}
	
	if (0 < carDrive->monitorDeviceValue(val, sizeof(val), UAVDRV_MONITORPORT_POSE_LOCAL))
	{
		data.push_back(val[0] * RADIAN); // steer angle
		data.push_back(val[1]);  // current forward(longitudinal) velocity
		data.push_back(val[2]);  // current lateral velocity
	}
	else
	{
		float nil(0.0f);
		data.push_back(nil);
		data.push_back(nil);
		data.push_back(nil);
	}
}

void SlipDataCallbackTracked(std::vector<double>& data)
{
	if (!sys) return;
	rxDevice* carDrive = sys->findDevice(_T("CarDrive"));
	if (!carDrive) return;
	float val[3];
	if (0 < carDrive->monitorDeviceValue(val, sizeof(val), UAVDRV_MONITORPORT_SLIP_ANGLE))
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

	if (0 < carDrive->monitorDeviceValue(val, sizeof(val), UAVDRV_MONITORPORT_VELOCITY))
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

void LateralForceDataCallback(std::vector<double>& data)
{
	if (!sys) return;
	rxDevice* carDrive = sys->findDevice(_T("CarDrive"));
	if (!carDrive) return;
	float val[2];
	carDrive->monitorDeviceValue(val, sizeof(val), UAVDRV_MONITORPORT_LATERAL_FORCE);
	data.push_back(val[0]);
	data.push_back(val[1]);
}

void SetupDAQ()
{
	vector<string_type> datanames;
	
	if (appConf.enablePlotPosition() | appConf.enablePlotVelocity())
	{
		rID pid_v = rdaqCreatePlot(_T("Vehicle"), eDataPlotType_TimeLine);
		rID did_v = rdaqAddData(pid_v, VehicleDataCallback);
		datanames.clear();
		if (appConf.enablePlotPosition()) {
			datanames.push_back(_T("x(m)"));
			datanames.push_back(_T("y(m)"));
			datanames.push_back(_T("yaw angle(deg)"));
			datanames.push_back(_T("pitch angle(deg)"));
			datanames.push_back(_T("roll angle(deg)"));
		}
		if (appConf.enablePlotVelocity()) {
			datanames.push_back(_T("vel_x(m/sec)"));
			datanames.push_back(_T("vel_y(m/sec)"));
			datanames.push_back(_T("yaw rate(deg/sec)"));
		}
		rdaqSetDataNames(did_v, datanames);
	}

	if (appConf.enablePlotSlippage())
	{
		rID pid_s = rdaqCreatePlot(_T("Slip"), eDataPlotType_TimeLine);
		rID did_s = rdaqAddData(pid_s, SlipDataCallback);
		datanames.clear();
		datanames.push_back(_T("beta slip angle(deg)"));
		datanames.push_back(_T("front slip angle(deg)"));
		datanames.push_back(_T("rear slip angle(deg)"));
		datanames.push_back(_T("steer angle(deg)"));
		datanames.push_back(_T("velocity(m/s)"));
		datanames.push_back(_T("lateral velocity(m/s)"));
		rdaqSetDataNames(did_s, datanames);
	}

	if (appConf.enablePlotSlippageTracked())
	{
		rID pid_s = rdaqCreatePlot(_T("Slip_Tracked"), eDataPlotType_TimeLine);
		rID did_s = rdaqAddData(pid_s, SlipDataCallbackTracked);
		datanames.clear();
		datanames.push_back(_T("slip angle(deg)"));
		datanames.push_back(_T("slip ratio(left, i_L)"));
		datanames.push_back(_T("slip ratio(left, i_R)"));
		datanames.push_back(_T("angular velocity(deg/s)"));
		datanames.push_back(_T("linear velocity in global x direction(m/s)"));
		datanames.push_back(_T("linear velocity in global y direction(m/s)"));
		rdaqSetDataNames(did_s, datanames);
	}

	if (appConf.enablePlotLateralForce())
	{
		rID pid_f = rdaqCreatePlot(_T("Lateral Force"), eDataPlotType_TimeLine);
		rID did_f = rdaqAddData(pid_f, LateralForceDataCallback);
		datanames.clear();
		datanames.push_back(_T("F_front(N)"));
		datanames.push_back(_T("F_rear(N)"));
		rdaqSetDataNames(did_f, datanames);
	}

	if (appConf.enablePlotGPSSensor())
	{
		rID pid_g = rdaqCreatePlot(_T("GPS"), eDataPlotType_TimeLine);
		rID did_g = rdaqAddData(pid_g, GPSDataCallback);
		datanames.clear();
		datanames.push_back(_T("longitude(m)"));
		datanames.push_back(_T("latitude(m)"));
		datanames.push_back(_T("altitude(m)"));
		rdaqSetDataNames(did_g, datanames);
	}

	if (appConf.enablePlotGyroSensor())
	{
		rID pid_g = rdaqCreatePlot(_T("IMU"), eDataPlotType_TimeLine);
		rID did_g = rdaqAddData(pid_g, GyroDataCallback);
		datanames.clear();
		datanames.push_back(_T("yaw angle(deg)"));
		datanames.push_back(_T("pitch angle(deg)"));
		datanames.push_back(_T("roll angle(deg)"));
		rdaqSetDataNames(did_g, datanames);
	}

	if (appConf.enablePlotAccSensor())
	{
		rID pid_g = rdaqCreatePlot(_T("Acceleration"), eDataPlotType_TimeLine);
		rID did_g = rdaqAddData(pid_g, AccDataCallback);
		datanames.clear();
		datanames.push_back(_T("acc_x(m/s^2)"));
		datanames.push_back(_T("acc_y(m/s^2)"));
		datanames.push_back(_T("acc_z(m/s^2)"));
		rdaqSetDataNames(did_g, datanames);
	}
}

void ParseCommand(int argc, _TCHAR* argv[])
{
	if (argc < 2) {
		PrintUsage(argc, argv);
		exit(-1);

		//aml_T0.r[0] = 0.0; aml_T0.r[1] = 0.0; aml_T0.r[2] = 0.75;
		//aml_T0.R.ZRotate(90*DEGREE);
	}

	appConf.parseConfig(argv[1]);
	delT = appConf.delT();
	eml_path = appConf.emlPath();
	eml_name = appConf.emlName();
	eml_T0.setIdentity();
	eml_T0.linear() = appConf.eml_R0();
	eml_T0.translation() = appConf.eml_r0();
	listenPort = appConf.netSimulationPort();
	bEnvironment = appConf.enableEml();

	for (int uav_no = 0; uav_no < MAX_UAV_NUM; uav_no++)
	{
		aml_path[uav_no] = appConf.amlPath(uav_no);
		aml_name[uav_no] = appConf.amlName(uav_no);
		aml_T0[uav_no].setIdentity();
		aml_T0[uav_no].linear() = appConf.aml_R0(uav_no);
		aml_T0[uav_no].translation() = appConf.aml_r0(uav_no);
		controlPort[uav_no] = appConf.netControlPort(uav_no);
	}
}

void PrintInstruction()
{
	_tprintf(_T("-----------------------------------------------------------\n"));
	_tprintf(_T("RoboticsLab SDK Version %s\n"), Version());
	_tprintf(_T("Copyright (C) Wonik Robotics. All rights reserved.\n"));
	_tprintf(_T("\n"));
	_tprintf(_T("Instructions:\n"));
	_tprintf(_T("\tUP arrow: increase velocity.\n"));
	_tprintf(_T("\tDOWN arrow: decrease velocity.\n"));
	_tprintf(_T("\tLEFT arrow: increase angular velocity in CCW direction.\n"));
	_tprintf(_T("\tRIGHT arrow: increase angular velocity in CW direction.\n"));
	_tprintf(_T("\tSpace-bar: stop.\n"));
	_tprintf(_T("\tPAGE UP: lift up PTO.\n"));
	_tprintf(_T("\tPAGE DOWN: lay down PTO.\n"));
	_tprintf(_T("\tT: show trace of the vehicle.\n"));
	//_tprintf(_T("\tTAB: start/pause simulation.\n"));
	_tprintf(_T("\tQ: quit this program.\n"));
	_tprintf(_T("\n"));
	_tprintf(_T("-----------------------------------------------------------\n"));
}

void PrintUsage(int argc, _TCHAR* argv[])
{
	_tprintf(_T("-----------------------------------------------------------\n"));
	_tprintf(_T("RoboticsLab SDK Version %s\n"), Version());
	_tprintf(_T("Copyright (C) Wonik Robotics. All rights reserved.\n"));
	_tprintf(_T("\n"));
	_tprintf(_T("\tUsage: %s <config.xdl>"), argv[0]);
	_tprintf(_T("\n"));
	_tprintf(_T("-----------------------------------------------------------\n"));
	_tprintf(_T("Press any key to continue...\n"));
	//_getch();
}
