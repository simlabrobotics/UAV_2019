/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#include <rlab/utils/rSampleUtil.h>
#include "rDeviceTractorDrive.h"
#include "rDeviceTractorDriveCmd.h"
#include "UAV_def.h"

#define A(i, j)		A_[3*(i) + (j)]
#define B(i, j)		B_[3*(i) + (j)]
#define At(i, j)	A_[3*(j) + (i)]
#define Bt(i, j)	B_[3*(j) + (i)]
#define C(i, j)		C_[3*(i) + (j)]

void MultMat(float* C_, const float* A_, const float* B_)
{
	assert(C_ != A_ && C_ != B_);
	memset(C_, 0, sizeof(float) * 9);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++)
				C(i, j) += A(i, k) * B(k, j);
		}
	}
}

void TrMultMat(float* C_, const float* A_, const float* B_)
{
	assert(C_ != A_ && C_ != B_);
	memset(C_, 0, sizeof(float) * 9);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++)
				C(i, j) += At(i, k) * B(k, j);
		}
	}
}

void MultMatTr(float* C_, const float* A_, const float* B_)
{
	assert(C_ != A_ && C_ != B_);
	memset(C_, 0, sizeof(float) * 9);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++)
				C(i, j) += A(i, k) * Bt(k, j);
		}
	}
}

void TrMultMatTr(float* C_, const float* A_, const float* B_)
{
	assert(C_ != A_ && C_ != B_);
	memset(C_, 0, sizeof(float) * 9);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++)
				C(i, j) += At(i, k) * Bt(k, j);
		}
	}
}

void MultVec(float* c, const float* A_, const float* b)
{
	assert(c != b);
	memset(c, 0, sizeof(float) * 3);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			c[i] += A(i, j) * b[j];
	}
}

void TrMultVec(float* c, const float* A_, const float* b)
{
	assert(c != b);
	memset(c, 0, sizeof(float) * 3);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			c[i] += At(i, j) * b[j];
	}
}

void AddVec(float* c, const float* a, const float* b)
{
	for (int i = 0; i < 3; i++)
		c[i] = a[i] + b[i];
}

void SubtractVec(float* c, const float* a, const float* b)
{
	for (int i = 0; i < 3; i++)
		c[i] = a[i] - b[i];
}

double NormAngleBtwPi(double x)
{
	x = fmod(x + M_PI, 2 * M_PI);
	if (x < 0) x += 2 * M_PI;
	return x - M_PI;
}

namespace rlab {
namespace plugin {

////////////////////////////////////////////////////////////////
// implementation of rDeviceCombineDrive
//
RD_IMPLE_FACTORY(TractorDrive)

rDeviceTractorDrive::rDeviceTractorDrive()
	: _nodeid(INVALID_RID)
	, _dT(0.001), _prevTime(0)
	, _distance_btw_axes(1)
	, _front_lwheel(INVALID_RID), _front_rwheel(INVALID_RID), _front_wheel_radius(1)
	, _rear_lwheel(INVALID_RID), _rear_rwheel(INVALID_RID), _rear_wheel_radius(1)
	, _steer_lwheel(INVALID_RID), _steer_rwheel(INVALID_RID)
	, _front_ltheta(0), _front_rtheta(0)
	, _rear_ltheta(0), _rear_rtheta(0)
	, _steer_ltheta(0), _steer_rtheta(0)
	, _steer_limit(-1), _steer_velocity_limit(-1)
	, _velocity_limit(-1), _acceleration_limit(-1)
	, _v_des(0), _theta_des(0), _v(0), _theta(0)
	, _vx(0), _vy(0)
	, _x(0), _y(0), _z(0), _psi(0), _pitch(0), _roll(0)
	, _xdot(0), _ydot(0), _zdot(0)
	, _psidot(0), _psiddot(0)
	, _beta(0), _betadot(0)
	, _betaf(0), _betar(0)
	, _Ff(0), _Fr(0)
	, _slip_ratio(0)
	, _v_threshod_to_apply_slippage(0.3f)
	, _hmap_normal_window_size(1)
{
	sample_init();
}

rDeviceTractorDrive::~rDeviceTractorDrive()
{
}

void rDeviceTractorDrive::onCreate(const rDeviceContext& rdc)
{
	RD_DEVICE_CLASS(Base)::onCreate(rdc);
}

void rDeviceTractorDrive::onInit()
{
	RD_DEVICE_CLASS(Base)::onInit();

	// load device parameters
	InitParams();
	PrintParams();

	// get tractor base body
	_nodeid = _rdc.m_deviceAPI->getBodyID(_rdc.m_robotname, _rdc.m_nodename);

	// get initial device frame
	//    R_G_C = R_G_V * R_V_C
	//    r_G_C = R_G_V * r_V_C + r_G_V
	//        where, 
	//            G := Global frame
	//            V := Vehicle frame
	//            C := COG of vehicle
	//
	float r0[3];
	float R0[9];
	_rdc.m_deviceAPI->getBodyPosition(_nodeid, r0);
	_rdc.m_deviceAPI->getBodyOrientation(_nodeid, R0);
	MultMat(_R0, R0, R);

	MultVec(_r0, R0, r);
	AddVec(_r0, _r0, r0);

	// initialize vehicle position and orientation
	_x = _r0[0];
	_y = _r0[1];
	_z = _r0[2];
	_psi = atan2(-_R0[1], _R0[0]);

	memset(_r, 0, sizeof(float) * 3);
	memset(_R, 0, sizeof(float) * 9);
	_R[0] = _R[4] = _R[8] = 1;

	_dT = (float)(period * 1e-6);

	_v = _v_des = 0;
	_theta = _theta_des = 0;
	_vx = _vy = 0;
	_beta = 0;
	_betadot = 0;
	_xdot = 0;
	_ydot = 0;
	_zdot = 0;
	_psidot = 0;
	_psiddot = 0;

	// initiate height map
	InitHeightMap();
}

void rDeviceTractorDrive::onTerminate()
{
	RD_DEVICE_CLASS(Base)::onTerminate();
}

int rDeviceTractorDrive::writeDeviceValue(void* buffer, int len, int port)
{
	switch (port)
	{
	case UAVDRV_DATAPORT_SET_POSE:
	{
		if (len >= 3 * sizeof(float))
		{
			_lock.lock();
			{
				float *value = (float*)buffer;
				_x = value[0];
				_y = value[1];
				_psi = value[2];

				_v = _v_des = 0;
				_theta = _theta_des = 0;
				_beta = 0;
				_betadot = 0;
				_xdot = 0;
				_ydot = 0;
				_zdot = 0;
				_psidot = 0;
				_psiddot = 0;
			}
			_lock.unlock();
			return 3 * sizeof(float);
		}
	}
	break;

	case UAVDRV_DATAPORT_SET_VEL_IMMEDIATE:
	{
		if (len >= 2 * sizeof(float))
		{
			_lock.lock();
			{
				float *value = (float*)buffer;
				_v = _v_des = value[0];		// forward velocity
				_theta = _theta_des = value[1];	// steer angle
			}
			_lock.unlock();
			return 2 * sizeof(float);
		}
	}
	break;

	case UAVDRV_DATAPORT_SET_VEL_TARGET:
	{
		if (len >= 2 * sizeof(float))
		{
			_lock.lock();
			{
				float *value = (float*)buffer;
				_v_des = value[0];		// forward velocity
				_theta_des = value[1];		// steer angle
			}
			_lock.unlock();
			return 2 * sizeof(float);
		}
	}

	default:
		// Exception! Unknown command.
		return 0;
	}

	return 0;
}

int rDeviceTractorDrive::monitorDeviceValue(void* buffer, int len, int port)
{
	switch (port)
	{
	case UAVDRV_MONITORPORT_POSE:
	{
		if (len >= 6 * sizeof(float))
		{
			_lock.lock();
			{
				float* value = (float*)buffer;
				value[0] = _x;
				value[1] = _y;
				value[2] = _z;
				value[3] = NormAngleBtwPi(_psi);
				value[4] = _pitch;
				value[5] = _roll;
			}
			_lock.unlock();
			return 6 * sizeof(float);
		}
		else if (len >= 5 * sizeof(float))
		{
			_lock.lock();
			{
				float* value = (float*)buffer;
				value[0] = _x;
				value[1] = _y;
				value[2] = NormAngleBtwPi(_psi);
				value[3] = _pitch;
				value[4] = _roll;
			}
			_lock.unlock();
			return 5 * sizeof(float);
		}
		else if (len >= 3 * sizeof(float))
		{
			_lock.lock();
			{
				float* value = (float*)buffer;
				value[0] = _x;
				value[1] = _y;
				value[2] = NormAngleBtwPi(_psi);
			}
			_lock.unlock();
			return 3 * sizeof(float);
		}
	}
	break;

	case UAVDRV_MONITORPORT_POSE_LOCAL:
	{
		if (len >= 3 * sizeof(float))
		{
			_lock.lock();
			{
				float* value = (float*)buffer;
				value[0] = _theta;
				value[1] = _vx;
				value[2] = _vy;
			}
			_lock.unlock();
			return 3 * sizeof(float);
		}
		else if (len >= 2 * sizeof(float))
		{
			_lock.lock();
			{
				float* value = (float*)buffer;
				value[0] = _theta;
				value[1] = _v;
			}
			_lock.unlock();
			return 2 * sizeof(float);
		}
	}
	break;

	case UAVDRV_MONITORPORT_VELOCITY:
	{
		if (len >= 3 * sizeof(float))
		{
			_lock.lock();
			{
				float* value = (float*)buffer;
				value[0] = _xdot;
				value[1] = _ydot;
				value[2] = _psidot;
			}
			_lock.unlock();
			return 3 * sizeof(float);
		}
	}
	break;

	case UAVDRV_MONITORPORT_SLIP_ANGLE:
	{
		if (len >= 3 * sizeof(float))
		{
			_lock.lock();
			{
				float* value = (float*)buffer;
				value[0] = _beta;
				value[1] = _betaf;
				value[2] = _betar;
			}
			_lock.unlock();
			return 3 * sizeof(float);
		}
	}
	break;

	case UAVDRV_MONITORPORT_LATERAL_FORCE:
	{
		if (len >= 2 * sizeof(float))
		{
			_lock.lock();
			{
				float* value = (float*)buffer;
				value[0] = _Ff;
				value[1] = _Fr;
			}
			_lock.unlock();
			return 2 * sizeof(float);
		}
	}
	break;

	default:
		// Exception! Unknown data port.
		return 0;
	}
	return 0;
}

void rDeviceTractorDrive::exportDevice(rTime time, void* mem)
{
	_lock.lock();
	{

		_dT = float(time - _prevTime);
		_prevTime = time;
		if (_dT < 1.0e-6)
		{
			_lock.unlock();
			return;
		}

		/////////////////////////////////////////////////////////////////////
		// calcurate current velocity(_v) considering velocity and acceleration limit
		if (_acceleration_limit > 0)
		{
			float vdot_des = (_v_des - _v) / _dT;
			if (vdot_des > _acceleration_limit)
				vdot_des = _acceleration_limit;
			else if (vdot_des < -_acceleration_limit)
				vdot_des = -_acceleration_limit;
			_v += vdot_des*_dT;
		}
		else
			_v = _v_des;

		if (_velocity_limit > 0)
			_v = RD_BOUND(_v, -_velocity_limit, _velocity_limit);

		/////////////////////////////////////////////////////////////////////
		// calcurate current steering angle(_theta) considering velocity and position limit
		if (_steer_velocity_limit > 0)
		{
			float thetadot_des = (_theta_des - _theta) / _dT;
			if (thetadot_des > _steer_velocity_limit)
				thetadot_des = _steer_velocity_limit;
			else if (thetadot_des < -_steer_velocity_limit)
				thetadot_des = -_steer_velocity_limit;
			_theta += thetadot_des*_dT;
		}
		else
			_theta = _theta_des;

		if (_steer_limit > 0)
			_theta = RD_BOUND(_theta, -_steer_limit, _steer_limit);

		/////////////////////////////////////////////////////////////////////
		// calcurate wheel velocity and angle
		float w_front_wheel = _v / _front_wheel_radius;
		float w_rear_wheel = _v / _rear_wheel_radius;

		_front_ltheta += w_front_wheel * _dT;
		_front_rtheta += w_front_wheel * _dT;

		_rear_ltheta += w_rear_wheel * _dT;
		_rear_rtheta += w_rear_wheel * _dT;

		_steer_ltheta = _theta;
		_steer_rtheta = _theta;

		/////////////////////////////////////////////////////////////////////
		// calcurate new position and orientation
		//
		if (_v >= _v_threshod_to_apply_slippage)
		{
			if (_Iz == 0 || _m == 0)
			{
				_vx = _v;
				_vy = 0;

				_xdot = _vx * cos(_psi);// - _vy * sin(_psi);
				_ydot = _vx * sin(_psi);// + _vy * cos(_psi);
				_psidot = _v * tan(_theta) / _distance_btw_axes;

				_x += _xdot * _dT;
				_y += _ydot * _dT;
				//_psi += _psidot * _dT;
				_psi += _psidot * _dT * (1.0f - _slip_ratio);
			}
			else
			{
				_vx = _v;
				_vy = 0;

				float A = -2 * (_Cr + _Cf) / (_m * _vx);
				float B = -1 - 2 * ((_Cf * _Lf - _Cr * _Lr) / (_m * _vx * _vx));
				float C = (2 * _Cf) / (_m * _vx);
				float D = -2 * ((_Cf * _Lf - _Cr * _Lr) / _Iz);
				float E = -2 * ((_Cf * _Lf * _Lf + _Cr * _Lr * _Lr) / (_Iz * _vx));
				float F = (2 * _Cf * _Lf) / _Iz;

				_betadot = (A * _beta) + (B * _psidot) + (C * _theta);
				_psiddot = (D * _beta) + (E * _psidot) + (F * _theta);

				_beta += _betadot * _dT;
				_psidot += _psiddot * _dT;
				_psi += _psidot  * _dT;

				float tbeta = tan(_beta);
				//float tbeta = _beta; // tan(th) ~= th when th << 1
				_vy = _vx * tbeta;

				_xdot = _vx*cos(_psi) - _vy*sin(_psi);
				_ydot = _vx*sin(_psi) + _vy*cos(_psi);

				_x += _xdot*_dT;
				_y += _ydot*_dT;

				// slip angles
				//////////////////////////////////////////////////////////////
				// from A DYNAMIC PATH SEARCH ALGORITHM FOR TRACTORI AUTOMATIC NAVIGATION
				//_betaf = _theta - atan2(_Lf * _psidot + _ydot, _xdot);
				//_betar = atan2(_Lr * _psidot - _ydot, _xdot);
				//////////////////////////////////////////////////////////////
				// from ROBUST ANTI-SLIDING CONTROL OF AUTONOMOUS VEHICLES IN PRESENCE OF LATERAL DISTURBANCES
				//_betaf = _theta - (_Lf*_psidot + _v*tan(_beta)) / _v;
				//_betar = (-_Lr*_psidot + _v*tan(_beta)) / _v;
				//////////////////////////////////////////////////////////////
				// from LATERAL VEHICLE DYNAMICS (2012 VDCL SNU)
				_betaf = _theta - (_Lf*_psidot + _vy) / _vx;
				_betar = (-_Lr*_psidot + _vy) / _vx;

				// lateral forces
				_Ff = _Cf * _betaf;
				_Fr = _Cr * _betar;
			}
		}
		else
		{
			// apply normal kinematics model without slippage.

			_beta = 0.0f;
			_betadot = 0.0f;
			_betaf = 0.0f;
			_betar = 0.0f;
			_Ff = 0.0f;
			_Fr = 0.0f;

			if (abs(_v) >= 1.0e-4)
			{
				_vx = _v;
				_vy = 0;

				_xdot = _vx * cos(_psi);// - _vy * sin(_psi);
				_ydot = _vx * sin(_psi);// + _vy * cos(_psi);
				_psidot = _v * tan(_theta) / _distance_btw_axes;

				_x += _xdot * _dT;
				_y += _ydot * _dT;
				//_psi += _psidot * _dT;
				_psi += _psidot * _dT * (1.0f - _slip_ratio);
			}
			else
			{
				_vx = _vy = 0;
				_xdot = 0;
				_ydot = 0;
				_zdot = 0;
				_psidot = 0;
				_psiddot = 0;
			}
		}

		// new vehicle(tractor) trasformation
		float c = cos(_psi);
		float s = sin(_psi);
		float height;
		_hmap.GetHeight(_x, _y, height);

		_R[0] = c;
		_R[1] = -s;
		_R[3] = s;
		_R[4] = c;
		_r[0] = _x;
		_r[1] = _y;
		//_r[2] = _z;
		_r[2] = height + _r0[2];

		// set model(base body) transformation
		//    T_G_V = T_G_C ^ T_V_C^-1
		// it leads to,
		//    R_G_V = R_G_C * R_V_C^-1
		//    r_G_V = -R_G_V * R_V_C^-1 * r_V_C + r_G_C
		//        where, 
		//            G := Global frame
		//            V := Vehicle frame
		//            C := COG of vehicle
		//
		float R1[9];
		float r1[3], r2[3];
		MultMatTr(R1, _R, R);
		MultVec(r2, R1, r);
		SubtractVec(r1, _r, r2);
		//_rdc.m_deviceAPI->setBodyHTransform(_nodeid, R1, r1);


		// apply face normal to model(robot) transform (pitch, roll):
		float normal[3];  // terrain face normal wrt global frame
		float normal2[3]; // terrain face normal wrt robot heading frame (R1_t * normal)
		float Ry[9], Rx[9];

		_hmap.GetNormalEx(_x, _y, _hmap_normal_window_size, normal);

		TrMultVec(normal2, R1, normal);

		_pitch = (float)(M_PI*0.5 - atan2(normal2[2], normal2[0]));
		c = cos(_pitch);
		s = sin(_pitch);
		Ry[0] = c; Ry[1] = 0; Ry[2] = s;
		Ry[3] = 0; Ry[4] = 1; Ry[5] = 0;
		Ry[6] = -s; Ry[7] = 0; Ry[8] = c;


		_roll = -(float)(M_PI*0.5 - atan2(sqrt(pow(normal2[0], 2) + pow(normal2[2], 2)), normal2[1]));
		c = cos(_roll);
		s = sin(_roll);
		Rx[0] = 1; Rx[1] = 0; Rx[2] = 0;
		Rx[3] = 0; Rx[4] = c; Rx[5] = -s;
		Rx[6] = 0; Rx[7] = s; Rx[8] = c;


		float R1_pitch[9];
		float R1_pitch_roll[9];

		MultMat(R1_pitch, R1, Ry);
		MultMat(R1_pitch_roll, R1_pitch, Rx);
		_rdc.m_deviceAPI->setBodyHTransform(_nodeid, R1_pitch_roll, r1);


		// visualizing rotating wheel and steer angles:
		if (_front_lwheel != INVALID_RID)
			_rdc.m_deviceAPI->setDeviceValue(_front_lwheel, &_front_ltheta, sizeof(float));
		if (_front_rwheel != INVALID_RID)
			_rdc.m_deviceAPI->setDeviceValue(_front_rwheel, &_front_rtheta, sizeof(float));
		if (_rear_lwheel != INVALID_RID)
			_rdc.m_deviceAPI->setDeviceValue(_rear_lwheel, &_rear_ltheta, sizeof(float));
		if (_rear_rwheel != INVALID_RID)
			_rdc.m_deviceAPI->setDeviceValue(_rear_rwheel, &_rear_rtheta, sizeof(float));
		if (_steer_lwheel != INVALID_RID)
			_rdc.m_deviceAPI->setDeviceValue(_steer_lwheel, &_steer_ltheta, sizeof(float));
		if (_steer_rtheta != INVALID_RID)
			_rdc.m_deviceAPI->setDeviceValue(_steer_rwheel, &_steer_rtheta, sizeof(float));
	}
	_lock.unlock();
}

// initialize device parameters
void rDeviceTractorDrive::InitParams()
{
	// initialize device parameters
	const TCHAR* distance_btw_axes = getProperty(_T("distance_btw_axes"));
	if (distance_btw_axes)
		_distance_btw_axes = (float)_tstof(distance_btw_axes);

	const TCHAR* front_radius = getProperty(_T("front_radius"));
	if (front_radius)
		_front_wheel_radius = (float)_tstof(front_radius);

	const TCHAR* front_lwheel = getProperty(_T("front_lwheel"));
	if (front_lwheel)
		_front_lwheel = _rdc.m_deviceAPI->getDeviceID(_rdc.m_robotname, front_lwheel);

	const TCHAR* front_rwheel = getProperty(_T("front_rwheel"));
	if (front_rwheel)
		_front_rwheel = _rdc.m_deviceAPI->getDeviceID(_rdc.m_robotname, front_rwheel);

	const TCHAR* rear_radius = getProperty(_T("rear_radius"));
	if (rear_radius)
		_rear_wheel_radius = (float)_tstof(rear_radius);

	const TCHAR* rear_lwheel = getProperty(_T("rear_lwheel"));
	if (rear_lwheel)
		_rear_lwheel = _rdc.m_deviceAPI->getDeviceID(_rdc.m_robotname, rear_lwheel);

	const TCHAR* rear_rwheel = getProperty(_T("rear_rwheel"));
	if (rear_rwheel)
		_rear_rwheel = _rdc.m_deviceAPI->getDeviceID(_rdc.m_robotname, rear_rwheel);

	const TCHAR* steer_lwheel = getProperty(_T("steer_lwheel"));
	if (steer_lwheel)
		_steer_lwheel = _rdc.m_deviceAPI->getDeviceID(_rdc.m_robotname, steer_lwheel);

	const TCHAR* steer_rwheel = getProperty(_T("steer_rwheel"));
	if (steer_rwheel)
		_steer_rwheel = _rdc.m_deviceAPI->getDeviceID(_rdc.m_robotname, steer_rwheel);

	const TCHAR* steer_limit = getProperty(_T("steer_limit"));
	if (steer_limit)
		_steer_limit = (float)(_tstof(steer_limit) * DEGREE);

	const TCHAR* steer_velocity_limit = getProperty(_T("steer_velocity_limit"));
	if (steer_velocity_limit)
		_steer_velocity_limit = (float)(_tstof(steer_velocity_limit) * DEGREE);

	const TCHAR* velocity_limit = getProperty(_T("velocity_limit"));
	if (velocity_limit)
		_velocity_limit = (float)_tstof(velocity_limit);

	const TCHAR* acceleration_limit = getProperty(_T("acceleration_limit"));
	if (acceleration_limit)
		_acceleration_limit = (float)_tstof(acceleration_limit);

	const TCHAR* Cf = getProperty(_T("Cf"));
	if (Cf)
		_Cf = (float)_tstof(Cf);

	const TCHAR* Cr = getProperty(_T("Cr"));
	if (Cr)
		_Cr = (float)_tstof(Cr);

	const TCHAR* Iz = getProperty(_T("Iz"));
	if (Iz)
		_Iz = (float)_tstof(Iz);

	const TCHAR* m = getProperty(_T("m"));
	if (m)
		_m = (float)_tstof(m);

	const TCHAR* Lf = getProperty(_T("Lf"));
	if (Lf)
		_Lf = (float)_tstof(Lf);

	const TCHAR* Lr = getProperty(_T("Lr"));
	if (Lr)
		_Lr = (float)_tstof(Lr);

	const TCHAR* slip_ratio = getProperty(_T("slip_ratio"));
	if (slip_ratio)
		_slip_ratio = (float)RD_BOUND(_tstof(slip_ratio), 0, 1.0);

	const TCHAR* v_threshold = getProperty(_T("velocity_threshold_to_apply_slippage"));
	if (v_threshold)
		_v_threshod_to_apply_slippage = (float)RD_LBOUND(_tstof(v_threshold), 1.0e-4);
}

void rDeviceTractorDrive::InitHeightMap()
{
	const TCHAR* prop;
	string_type hmap_path = _T("");
	float hmap_map_width = 0.0f;
	float hmap_map_length = 0.0f;
	float hmap_map_height_min = 0.0f;
	float hmap_map_height_max = 0.0f;

	prop = getProperty(_T("HMAP_PATH")); if (prop) hmap_path = prop;
	prop = getProperty(_T("HMAP_W")); if (prop) hmap_map_width = (float)_tstof(prop);
	prop = getProperty(_T("HMAP_L")); if (prop) hmap_map_length = (float)_tstof(prop);
	prop = getProperty(_T("HMAP_H_min")); if (prop) hmap_map_height_min = (float)_tstof(prop);
	prop = getProperty(_T("HMAP_H_max")); if (prop) hmap_map_height_max = (float)_tstof(prop);
	prop = getProperty(_T("HMAP_NORMAL_WND_SIZE")); if (prop) _hmap_normal_window_size = _tstoi(prop);

	if (!_hmap.Create(hmap_path, hmap_map_width, hmap_map_length, hmap_map_height_min, hmap_map_height_max))
	{
		assert(0 && "ERROR! rDeviceTractorDrive: failed to load height map.\n");
	}
}

void rDeviceTractorDrive::PrintParams()
{
	printf("********** rDeviceTractorDrive ********\n");
	printf("\n\t== VEHICLE KINEMATICS ==\n\n");
	printf("Distance between front and rear axis: \t%.3f \t(m)\n", _distance_btw_axes);
	printf("Radius of front wheel: \t%.3f \t(m)\n", _front_wheel_radius);
	printf("Radius of rear wheel: \t%.3f \t(m)\n", _rear_wheel_radius);
	printf("Steering angle limit: \t%.3f \t(degree)\n", _steer_limit*RADIAN);
	printf("Steering velocity limit: \t%.3f \t(degree/s)\n", _steer_velocity_limit*RADIAN);
	printf("Longitudinal velocity limit: \t%.3f \t(m/s)\n", _velocity_limit);
	printf("Longitudinal acceleration limit: \t%.3f \t(m/s^2)\n", _acceleration_limit);
	printf("\n\t== VEHICLE DYNAMICS ==\n\n");
	printf("Cornering stiffness of front wheels(Cf): \t%.3f \t(N/rad)\n", _Cf);
	printf("Cornering stiffness of rear wheels(Cr): \t%.3f \t(N/rad)\n", _Cr);
	printf("Mass of the tractor(m): \t%.3f \t(kg)\n", _m);
	printf("Turning inertia WRT tractor COG(Iz): \t%.3f \t(kg m^2)\n", _Iz);
	printf("Distance from front wheel to tractor COG(Lf): \t%.3f \t(m)\n", _Lf);
	printf("Distance from rear wheel to tractor COG(Lr): \t%.3f \t(m)\n", _Lr);
	printf("\n");
	printf("\n");
	printf("***************************************\n");
}

} // namespace plugin
} // namespace rlab
