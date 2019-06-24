/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#include <rlab/utils/rSampleUtil.h>
#include "rDeviceCombineDrive.h"
#include "rDeviceCombineDriveCmd.h"
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


namespace rlab {
namespace plugin {

////////////////////////////////////////////////////////////////
// implementation of rDeviceCombineDrive
//
RD_IMPLE_FACTORY(CombineDrive)

rDeviceCombineDrive::rDeviceCombineDrive() 
: _nodeid(INVALID_RID)
, _dT(0.001f), _prevTime(0)
, _front_lwheel(INVALID_RID), _front_rwheel(INVALID_RID)
, _rear_lwheel(INVALID_RID), _rear_rwheel(INVALID_RID)
, _front_lwheel_theta(0), _front_rwheel_theta(0)
, _rear_lwheel_theta(0), _rear_rwheel_theta(0)
, _thread_length(1), _front_wheel_radius(1), _rear_wheel_radius(1), _driving_wheel_radius(1)
, _w_limit(-1), _wdot_limit(-1)
, _w_l_des(0), _w_r_des(0)
, _w_l(0), _w_r(0)
, _w_l_w_noise(0), _w_r_w_noise(0)
, _v(0), _thdot(0)
, _v_wo_slip(0), _thdot_wo_slip(0)
, _gx(0), _gy(0), _gz(0), _gth(0), _pitch(0), _roll(0)
, _gxdot(0), _gydot(0), _gzdot(0), _gthdot(0), _gthddot(0)
, _slip_ratio_mean(0), _slip_ratio_std(0)
, _w_l_noise_mean(0), _w_l_noise_std(0)
, _w_r_noise_mean(0), _w_r_noise_std(0)
, _i_r(0), _i_l(0)
, _mu_l(1), _gravity_acc(9.8)
, _alpha(0)
, _v_threshod_to_apply_slippage(0.3f)
, _hmap_normal_window_size(1)
, _K(1)
, _pi(1)
, _mass(1)
, _track_width(1)
, _track_length(1)
, _k_c(1)
, _k_pi(1)
, _n(1)
, _F_r(0), _F_l(0)
, _z0(0)
, _Rc(0)
{
	sample_init();
}

rDeviceCombineDrive::~rDeviceCombineDrive()
{
}

void rDeviceCombineDrive::onCreate(const rDeviceContext& rdc)
{
	RD_DEVICE_CLASS(Base)::onCreate(rdc);
}

void rDeviceCombineDrive::onInit()
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
	_gx = _r0[0];
	_gy = _r0[1];
	_gz = _r0[2];
	_gth = atan2(-_R0[1], _R0[0]);

	memset(_r, 0, sizeof(float)*3);
	memset(_R, 0, sizeof(float)*9);
	_R[0] = _R[4] = _R[8] = 1;
	
	_dT = (float)(period * 1e-6);

	_w_l = _w_l_des = 0;
	_w_r = _w_r_des = 0;
	_v = 0;
	_thdot = 0;
	_v_wo_slip = 0;
	_thdot_wo_slip = 0;
	_gxdot = 0;
	_gydot = 0;
	_gzdot = 0;
	_gthdot = 0;
	_gthddot = 0;
	_i_r = _i_l = 0;
	_alpha = 0;

	// initiate height map
	InitHeightMap();
}

void rDeviceCombineDrive::onTerminate()
{
	RD_DEVICE_CLASS(Base)::onTerminate();
}

int rDeviceCombineDrive::writeDeviceValue(void* buffer, int len, int port)
{
	switch (port)
	{
	case UAVDRV_DATAPORT_SET_POSE: // set instant pose of the vehicle
		{
			if (len >= 3*sizeof(float))
			{
				_lock.lock();
				{
					float *value = (float*)buffer;
					_gx = value[0];
					_gy = value[1];
					_gth = value[2];

					_w_l = _w_l_des = 0;
					_w_r = _w_r_des = 0;
					_v = 0;
					_thdot = 0;
					_v_wo_slip = 0;
					_thdot_wo_slip = 0;
					_gxdot = 0;
					_gydot = 0;
					_gzdot = 0;
					_gthdot = 0;
					_gthddot = 0;
				}
				_lock.unlock();
				return 3*sizeof(float);
			}
		}
		break;

	case UAVDRV_DATAPORT_SET_VEL_IMMEDIATE:
		{
			if (len >= 2*sizeof(float))
			{
				_lock.lock();
				{
					float *value = (float*)buffer;
					_w_l = _w_l_des = value[0];	// left wheel velocity
					_w_r = _w_r_des = value[1];	// right wheel velocity
				}
				_lock.unlock();
				return 2*sizeof(float);
			}
		}
		break;

	case UAVDRV_DATAPORT_SET_MAXIMUM_VEL:
		{
			if (len >= 2*sizeof(float))
			{
				_lock.lock();
				{
					float *value = (float*)buffer;
					if (value[0] > 1.0e-6)
						_w_limit = value[0];	// maximum wheel(sprocket) velocity
					else
						_w_limit = -1.0f;
					if (value[1] > 1.0e-6)
						_wdot_limit = value[1];	// maximum wheel(sprocket) acceleration
					else
						_wdot_limit = -1.0f;
				}
				_lock.unlock();
				return 2*sizeof(float);
			}
		}
		break;

	case UAVDRV_DATAPORT_SET_PARAMETER:
		{
			if (len >= 4*sizeof(float))
			{
				_lock.lock();
				{
					int *ivalue = (int*)buffer;
					float *fvalue = (float*)buffer;
					switch (ivalue[0])
					{
					case PTYPE_lateral_resistance:
						if (fvalue[1] > 0.0f) _mu_l = fvalue[1];
						break;
					case PTYPE_gravitational_acceleration:
						if (fvalue[1] > 0.0f) _gravity_acc = fvalue[1];
						break;
					case PTYPE_slip_ratio_mean_and_std:
						_slip_ratio_mean = fvalue[1];
						_slip_ratio_std  = fvalue[2];
						break;
					case PTYPE_maximum_wheel_angular_velocity:
						if (fvalue[1] > 0.0f) _w_limit = fvalue[1];
						break;
					case PTYPE_maximum_wheel_angular_acceleration:
						if (fvalue[1] > 0.0f) _wdot_limit = fvalue[1];
						break;
					case PTYPE_velocity_threshold_to_apply_slippage:
						if (fvalue[1] > 0.0f) _v_threshod_to_apply_slippage = fvalue[1];
						break;
					case PTYPE_wheel_velocity_noise_mean_and_std:
						_w_l_noise_mean = fvalue[1];
						_w_l_noise_std  = fvalue[2];
						_w_r_noise_mean = fvalue[1];
						_w_r_noise_std  = fvalue[2];
						break;
					case PTYPE_deformation_modulus:
						if (fvalue[1] > 0.0f) _K = fvalue[1];
						break;
					case PTYPE_internal_shearing_resistance:
						if (fvalue[1] > 0.0f) _pi = fvalue[1];
						break;
					case PTYPE_cohesive_modulus_of_terrain_deformation:
						if (fvalue[1] > 0.0f) _k_c = fvalue[1];
						break;
					case PTYPE_frictional_modulus_of_terrain_deformation:
						if (fvalue[1] > 0.0f) _k_pi = fvalue[1];
						break;
					case PTYPE_exponent_of_terrain_deformation:
						if (fvalue[1] > 0.0f) _n = fvalue[1];
						break;
					case PTYPE_tread_length:
						if (fvalue[1] > 0.0f) _thread_length = fvalue[1];
						break;
					case PTYPE_track_length:
						if (fvalue[1] > 0.0f) _track_length = fvalue[1];
						break;
					case PTYPE_track_width:
						if (fvalue[1] > 0.0f) _track_width = fvalue[1];
						break;
					case PTYPE_vehicle_mass:
						if (fvalue[1] > 0.0f) _mass = fvalue[1];
						break;
					case PTYPE_wheel_radius:
						if (fvalue[1] > 0.0f) _driving_wheel_radius = fvalue[1];
						break;
					}
				}
				_lock.unlock();
				return 4*sizeof(float);
			}
		}
		break;

	case UAVDRV_DATAPORT_SET_VEL_TARGET: // set velocity command
		{
			if (len >= 2*sizeof(float))
			{
				_lock.lock();
				{
					float *value = (float*)buffer;
					_w_l_des = value[0];	// left wheel velocity
					_w_r_des = value[1];	// right wheel velocity
				}
				_lock.unlock();
				return 2*sizeof(float);
			}
		}
		break;

	default:
		// Exception! Unknown command.
		return 0;
	}

	return 0;
}

int rDeviceCombineDrive::monitorDeviceValue(void* buffer, int len, int port)
{
	switch (port)
	{
	case UAVDRV_MONITORPORT_SLIP_ANGLE:
		{
			if (len >= 3*sizeof(float))
			{
				_lock.lock();
				{
					float* value = (float*)buffer;
					value[0] = _i_l;
					value[1] = _i_r;
					value[2] = _alpha;
				}
				_lock.unlock();
				return 3*sizeof(float);
			}
		}
		break;

	case UAVDRV_MONITORPORT_VELOCITY_LOCAL:
		{
			if (len >= 2*sizeof(float))
			{
				_lock.lock();
				{
					float* value = (float*)buffer;
					value[0] = _v;
					value[1] = _thdot;
				}
				_lock.unlock();
				return 2*sizeof(float);
			}
		}
		break;

	case UAVDRV_MONITORPORT_VELOCITY_LOCAL_WO_SLIP:
		{
			if (len >= 2*sizeof(float))
			{
				_lock.lock();
				{
					float* value = (float*)buffer;
					value[0] = _v_wo_slip;
					value[1] = _thdot_wo_slip;
				}
				_lock.unlock();
				return 2*sizeof(float);
			}
		}
		break;

	case UAVDRV_MONITORPORT_VELOCITY:
		{
			if (len >= 3*sizeof(float))
			{
				_lock.lock();
				{
					float* value = (float*)buffer;
					value[0] = _gxdot;
					value[1] = _gydot;
					value[2] = _gthdot;
				}
				_lock.unlock();
				return 3*sizeof(float);
			}
		}
		break;

	case UAVDRV_MONITORPORT_WHEEL_VELOCITY:
		{
			if (len >= 2*sizeof(float))
			{
				_lock.lock();
				{
					float* value = (float*)buffer;
					value[0] = _w_l_w_noise;
					value[1] = _w_r_w_noise;
				}
				_lock.unlock();
				return 2*sizeof(float);
			}
		}
		break;

	case UAVDRV_MONITORPORT_WHEEL_VELOCITY_WO_SLIP:
		{
			if (len >= 2*sizeof(float))
			{
				_lock.lock();
				{
					float* value = (float*)buffer;
					value[0] = _w_l;
					value[1] = _w_r;
				}
				_lock.unlock();
				return 2*sizeof(float);
			}
		}
		break;

	case UAVDRV_MONITORPORT_TRACTIVE_FORCE:
		{
			if (len >= 2*sizeof(float))
			{
				_lock.lock();
				{
					float* value = (float*)buffer;
					value[0] = _F_l;
					value[1] = _F_r;
				}
				_lock.unlock();
				return 2*sizeof(float);
			}
		}
		break;

	case UAVDRV_MONITORPORT_SINKAGE:
		{
			if (len >= sizeof(float))
			{
				_lock.lock();
				{
					float* value = (float*)buffer;
					value[0] = _z0;
				}
				_lock.unlock();
				return sizeof(float);
			}
		}
		break;

	case UAVDRV_MONITORPORT_MOTION_RESISTANCE:
		{
			if (len >= sizeof(float))
			{
				_lock.lock();
				{
					float* value = (float*)buffer;
					value[0] = _Rc;
				}
				_lock.unlock();
				return sizeof(float);
			}
		}
		break;

	case UAVDRV_MONITORPORT_PARAMETER:
		{
			if (len >= 4*sizeof(float))
			{
				_lock.lock();
				{
					int *ivalue = (int*)buffer;
					float *fvalue = (float*)buffer;
					switch (ivalue[0])
					{
					case PTYPE_lateral_resistance:
						fvalue[1] = _mu_l;
						break;
					case PTYPE_gravitational_acceleration:
						fvalue[1] = _gravity_acc;
						break;
					case PTYPE_slip_ratio_mean_and_std:
						fvalue[1] = _slip_ratio_mean;
						fvalue[2] = _slip_ratio_std;
						break;
					case PTYPE_maximum_wheel_angular_velocity:
						fvalue[1] = _w_limit;
						break;
					case PTYPE_maximum_wheel_angular_acceleration:
						fvalue[1] = _wdot_limit;
						break;
					case PTYPE_velocity_threshold_to_apply_slippage:
						fvalue[1] = _v_threshod_to_apply_slippage;
						break;
					case PTYPE_wheel_velocity_noise_mean_and_std:
						fvalue[1] = _w_l_noise_mean;
						fvalue[2] = _w_l_noise_std;
						//fvalue[1] = _w_r_noise_mean;
						//fvalue[2] = _w_r_noise_std;
						break;
					case PTYPE_deformation_modulus:
						fvalue[1] = _K;
						break;
					case PTYPE_internal_shearing_resistance:
						fvalue[1] = _pi;
						break;
					case PTYPE_cohesive_modulus_of_terrain_deformation:
						fvalue[1] = _k_c;
						break;
					case PTYPE_frictional_modulus_of_terrain_deformation:
						fvalue[1] = _k_pi;
						break;
					case PTYPE_exponent_of_terrain_deformation:
						fvalue[1] = _n;
						break;
					case PTYPE_tread_length:
						fvalue[1] = _thread_length;
						break;
					case PTYPE_track_length:
						fvalue[1] = _track_length;
						break;
					case PTYPE_track_width:
						fvalue[1] = _track_width;
						break;
					case PTYPE_vehicle_mass:
						fvalue[1] = _mass;
						break;
					case PTYPE_wheel_radius:
						fvalue[1] = _driving_wheel_radius;
						break;
					}
				}
				_lock.unlock();
				return 4*sizeof(float);
			}
		}
		break;

	case UAVDRV_MONITORPORT_POSE:
		{
			if (len >= 5*sizeof(float))
			{
				float* value = (float*)buffer;
					value[0] = _gx;
					value[1] = _gy;
					value[2] = _gth;
					value[3] = _pitch;
					value[4] = _roll;
			}
			else if (len >= 3*sizeof(float))
			{
				_lock.lock();
				{
					float* value = (float*)buffer;
					value[0] = _gx;
					value[1] = _gy;
					value[2] = _gth;
				}
				_lock.unlock();
				return 3*sizeof(float);
			}
		}
		break;

	default:
		// Exception! Unknown data port.
		return 0;
	}
	return 0;
}

double tractiveForce(double i, double pi, double K, double l, double W)
{
	static double e = 2.7182818284590452353602874;
	double il_over_K = i * l / K;
	double e_pow = pow(e, -il_over_K);
	double W_tan = W * tan(pi);
	double F = 2 * W_tan * (1 - 1 / il_over_K * (1 - e_pow)) - W_tan * (1 - pow(1/il_over_K, 2) * (1 - e_pow - il_over_K * e_pow));
	return F;
}

void rDeviceCombineDrive::exportDevice(rTime time, void* mem)
{
	_lock.lock();
	{
	
		_dT = float(time - _prevTime);
		_prevTime = time;
		if (_dT < 1.0e-6) {
			_lock.unlock();
			return;
		}

		/////////////////////////////////////////////////////////////////////
		// calcurate current wheel velocity considering velocity and acceleration limit
		if (_wdot_limit > 0) {
			float wdot_l_des = (_w_l_des - _w_l) / _dT;
			if (wdot_l_des > _wdot_limit)
				wdot_l_des = _wdot_limit;
			else if (wdot_l_des < -_wdot_limit)
				wdot_l_des = -_wdot_limit;
			_w_l += wdot_l_des * _dT;

			float wdot_r_des = (_w_r_des - _w_r) / _dT;
			if (wdot_r_des > _wdot_limit)
				wdot_r_des = _wdot_limit;
			else if (wdot_r_des < -_wdot_limit)
				wdot_r_des = -_wdot_limit;
			_w_r += wdot_r_des * _dT;
		}
		else {
			_w_l = _w_l_des;
			_w_r = _w_r_des;
		}

		if (_w_limit > 0) {
			_w_l = RD_BOUND(_w_l, -_w_limit, _w_limit);
			_w_r = RD_BOUND(_w_r, -_w_limit, _w_limit);
		}

		if (abs(_w_l) < 1.0e-4)
			_w_l = 0.0;
		if (abs(_w_r) < 1.0e-4)
			_w_r = 0.0;

		/////////////////////////////////////////////////////////////////////
		// add some noise to wheel velocity
		_w_l_w_noise = _w_l + sample_quasi_normal(_w_l_noise_mean, _w_l_noise_std);
		_w_r_w_noise = _w_r + sample_quasi_normal(_w_r_noise_mean, _w_r_noise_std);

		/////////////////////////////////////////////////////////////////////
		// calcurate slip ratio & slip angle
		_i_l = sample_quasi_normal(_slip_ratio_mean, _slip_ratio_std);
		_i_l = RD_BOUND(_i_l, 0.0, 1.0);
		_i_r = sample_quasi_normal(_slip_ratio_mean, _slip_ratio_std);
		_i_r = RD_BOUND(_i_r, 0.0, 1.0);
		_alpha = atan(_gthdot * _gthdot * _thread_length * 0.25 / _gravity_acc / _mu_l);
		if (_gthdot < 0) _alpha *= -1.0f; // slip ratio should have the same sign with yaw rate

		/////////////////////////////////////////////////////////////////////
		// calcurate linear & angular velocities of the vehicle 
		_v_wo_slip = _driving_wheel_radius * 0.5 * (_w_l_w_noise + _w_r_w_noise);
		_thdot_wo_slip = -_driving_wheel_radius / _thread_length * (_w_l_w_noise - _w_r_w_noise);

		/////////////////////////////////////////////////////////////////////
		// calcurate linear & angular velocities of the vehicle 
		// with slip ratio & slip angle
		if (_v_wo_slip >= _v_threshod_to_apply_slippage) {
			_v = _driving_wheel_radius * 0.5 * (_w_l_w_noise * (1.0 - _i_l) + _w_r_w_noise * (1.0 - _i_r));
			_thdot = -_driving_wheel_radius / _thread_length * (_w_l_w_noise * (1.0 - _i_l) - _w_r_w_noise * (1.0 - _i_r));
			_gxdot = _v * (cos(_gth) - sin(_gth) * tan(_alpha));
			_gydot = _v * (sin(_gth) - cos(_gth) * tan(_alpha));
		}
		else {
			// for the backward direction, apply normal kinematics model
			_v = _v_wo_slip;
			_thdot = _thdot_wo_slip;
			_gxdot = _v * cos(_gth);
			_gydot = _v * sin(_gth);
		}
		_gthddot = (_thdot - _gthdot) / _dT;
		_gthdot = _thdot;

		/////////////////////////////////////////////////////////////////////
		// calcurate passive wheel velocity and angle
		_front_lwheel_theta += _w_l_w_noise * (_driving_wheel_radius / _front_wheel_radius) * _dT;
		_front_rwheel_theta += _w_r_w_noise * (_driving_wheel_radius / _front_wheel_radius) * _dT;
		_rear_lwheel_theta += _w_l_w_noise * (_driving_wheel_radius / _rear_wheel_radius) * _dT;
		_rear_rwheel_theta += _w_r_w_noise * (_driving_wheel_radius / _rear_wheel_radius) * _dT;

		/////////////////////////////////////////////////////////////////////
		// calcurate new position and orientation
		_gx += _gxdot * _dT;
		_gy += _gydot * _dT;
		_gth += _gthdot * _dT;

		/////////////////////////////////////////////////////////////////////
		// calculate new vehicle pose
		float c = cos(_gth);
		float s = sin(_gth);
		float height;
		_hmap.GetHeight(_gx, _gy, height);

		_R[0] = c;
		_R[1] = -s;
		_R[3] = s;
		_R[4] = c;
		_r[0] = _gx;
		_r[1] = _gy;
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
		
		_hmap.GetNormalEx(_gx, _gy, _hmap_normal_window_size, normal);
		
		TrMultVec(normal2, R1, normal);
		
		_pitch = (float)(M_PI*0.5 - atan2(normal2[2], normal2[0]));
		c = cos(_pitch);
		s = sin(_pitch);
		Ry[0] = c; Ry[1] = 0; Ry[2] = s;
		Ry[3] = 0; Ry[4] = 1; Ry[5] = 0;
		Ry[6] =-s; Ry[7] = 0; Ry[8] = c;


		_roll = -(float)(M_PI*0.5 - atan2(sqrt(pow(normal2[0],2) + pow(normal2[2],2)), normal2[1]));
		c = cos(_roll);
		s = sin(_roll);
		Rx[0] = 1; Rx[1] = 0; Rx[2] = 0;
		Rx[3] = 0; Rx[4] = c; Rx[5] =-s;
		Rx[6] = 0; Rx[7] = s; Rx[8] = c;


		float R1_pitch[9];
		float R1_pitch_roll[9];

		MultMat(R1_pitch, R1, Ry);
		MultMat(R1_pitch_roll, R1_pitch, Rx);
		_rdc.m_deviceAPI->setBodyHTransform(_nodeid, R1_pitch_roll, r1);


		// visualizing rotating wheel and steer angles:
		if (_front_lwheel != INVALID_RID)
			_rdc.m_deviceAPI->setDeviceValue(_front_lwheel, &_front_lwheel_theta, sizeof(float));
		if (_front_rwheel != INVALID_RID)
			_rdc.m_deviceAPI->setDeviceValue(_front_rwheel, &_front_rwheel_theta, sizeof(float));
		if (_rear_lwheel  != INVALID_RID)
			_rdc.m_deviceAPI->setDeviceValue(_rear_lwheel, &_rear_lwheel_theta, sizeof(float));
		if (_rear_rwheel  != INVALID_RID)
			_rdc.m_deviceAPI->setDeviceValue(_rear_rwheel, &_rear_rwheel_theta, sizeof(float));

		/////////////////////////////////////////////////////////////////////
		// calculate additional informative indicators
		// tractive force of left track:
		_F_r = (float)tractiveForce(_i_r, _pi, _K, _track_length, _mass*_gravity_acc);
		_F_l = (float)tractiveForce(_i_l, _pi, _K, _track_length, _mass*_gravity_acc);
	
		// sinkage:
		_z0 = (float)pow(_mass * _gravity_acc / (2 * _track_length * _track_width * (_k_c / _track_width + _k_pi)), 1/_n);

		// motion resistance:
		_Rc = (float)(2 * _track_width * (_k_c / _track_width + _k_pi) * pow(_z0, (_n + 1)) / (_n + 1));
	}
	_lock.unlock();
}

void rDeviceCombineDrive::ComputeIndicators()
{

}

// initialize device parameters
void rDeviceCombineDrive::InitParams()
{
	const TCHAR* front_lwheel = getProperty(_T("front_lwheel"));
	if (front_lwheel)
		_front_lwheel = _rdc.m_deviceAPI->getDeviceID(_rdc.m_robotname, front_lwheel);

	const TCHAR* front_rwheel = getProperty(_T("front_rwheel"));
	if (front_rwheel)
		_front_rwheel = _rdc.m_deviceAPI->getDeviceID(_rdc.m_robotname, front_rwheel);

	const TCHAR* rear_lwheel = getProperty(_T("rear_lwheel"));
	if (rear_lwheel)
		_rear_lwheel = _rdc.m_deviceAPI->getDeviceID(_rdc.m_robotname, rear_lwheel);

	const TCHAR* rear_rwheel = getProperty(_T("rear_rwheel"));
	if (rear_rwheel)
		_rear_rwheel = _rdc.m_deviceAPI->getDeviceID(_rdc.m_robotname, rear_rwheel);

	const TCHAR* thread_length = getProperty(_T("thread_length"));
	if (thread_length)
		_thread_length = (float)_tstof(thread_length);

	const TCHAR* front_wheel_radius = getProperty(_T("front_wheel_radius"));
	if (front_wheel_radius)
		_front_wheel_radius = (float)_tstof(front_wheel_radius);

	const TCHAR* rear_wheel_radius = getProperty(_T("rear_wheel_radius"));
	if (rear_wheel_radius)
		_rear_wheel_radius = (float)_tstof(rear_wheel_radius);

	const TCHAR* driving_wheel_radius = getProperty(_T("driving_wheel_radius"));
	if (driving_wheel_radius)
		_driving_wheel_radius = (float)_tstof(driving_wheel_radius);

	const TCHAR* wheel_angular_vel_limit = getProperty(_T("wheel_angular_vel_limit"));
	if (wheel_angular_vel_limit)
		_w_limit = (float)_tstof(wheel_angular_vel_limit);

	const TCHAR* wheel_angular_acc_limit = getProperty(_T("wheel_angular_acc_limit"));
	if (wheel_angular_acc_limit)
		_wdot_limit = (float)_tstof(wheel_angular_acc_limit);

	const TCHAR* slip_ratio_coef_mean = getProperty(_T("slip_ratio_coef_mean"));
	if (slip_ratio_coef_mean)
		_slip_ratio_mean = (float)_tstof(slip_ratio_coef_mean);

	const TCHAR* slip_ratio_coef_std = getProperty(_T("slip_ratio_coef_std"));
	if (slip_ratio_coef_std)
		_slip_ratio_std = (float)_tstof(slip_ratio_coef_std);

	const TCHAR* lateral_resistance = getProperty(_T("lateral_resistance"));
	if (lateral_resistance)
		_mu_l = (float)RD_BOUND(_tstof(lateral_resistance), 0, 1.0);

	const TCHAR* gravity_acc = getProperty(_T("gravity_acc"));
	if (gravity_acc)
		_gravity_acc = (float)_tstof(gravity_acc);

	const TCHAR* v_threshold = getProperty(_T("velocity_threshold_to_apply_slippage"));
	if (v_threshold)
		_v_threshod_to_apply_slippage = (float)RD_LBOUND(_tstof(v_threshold), 1.0e-4);

	const TCHAR* lwheel_velocity_noise_mean = getProperty(_T("lwheel_velocity_noise_mean"));
	if (lwheel_velocity_noise_mean)
		_w_l_noise_mean = (float)_tstof(lwheel_velocity_noise_mean);

	const TCHAR* lwheel_velocity_noise_std = getProperty(_T("lwheel_velocity_noise_std"));
	if (lwheel_velocity_noise_std)
		_w_l_noise_std = (float)_tstof(lwheel_velocity_noise_std);

	const TCHAR* rwheel_velocity_noise_mean = getProperty(_T("rwheel_velocity_noise_mean"));
	if (rwheel_velocity_noise_mean)
		_w_r_noise_mean = (float)_tstof(rwheel_velocity_noise_mean);

	const TCHAR* rwheel_velocity_noise_std = getProperty(_T("rwheel_velocity_noise_std"));
	if (rwheel_velocity_noise_std)
		_w_r_noise_std = (float)_tstof(rwheel_velocity_noise_std);

	const TCHAR* deformation_modulus = getProperty(_T("deformation_modulus"));
	if (deformation_modulus)
		_K = (float)_tstof(deformation_modulus);

	const TCHAR* internal_shearing_resistance = getProperty(_T("internal_shearing_resistance"));
	if (internal_shearing_resistance)
		_pi = (float)_tstof(internal_shearing_resistance);

	const TCHAR* mass = getProperty(_T("mass"));
	if (mass)
		_mass = (float)_tstof(mass);

	const TCHAR* track_width = getProperty(_T("track_width"));
	if (track_width)
		_track_width = (float)_tstof(track_width);

	const TCHAR* track_length = getProperty(_T("track_length"));
	if (track_length)
		_track_length = (float)_tstof(track_length);

	const TCHAR* cohesive_modulus_of_terrain_deformation = getProperty(_T("cohesive_modulus_of_terrain_deformation"));
	if (cohesive_modulus_of_terrain_deformation)
		_k_c = (float)_tstof(cohesive_modulus_of_terrain_deformation);

	const TCHAR* frictional_modulus_of_terrain_deformation = getProperty(_T("frictional_modulus_of_terrain_deformation"));
	if (frictional_modulus_of_terrain_deformation)
		_k_pi = (float)_tstof(frictional_modulus_of_terrain_deformation);

	const TCHAR* exponent_of_terrain_deformation = getProperty(_T("exponent_of_terrain_deformation"));
	if (exponent_of_terrain_deformation)
		_n = (float)_tstof(exponent_of_terrain_deformation);
}

void rDeviceCombineDrive::InitHeightMap()
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
		assert(0 && "ERROR! rDeviceCombineDrive: failed to load height map.\n");
	}
}

void rDeviceCombineDrive::PrintParams()
{
	printf("********** rDeviceCombineDrive ********\n");

	printf("\n\t==   VEHICLE KINEMATICS    ==\n\n");
	printf("Distance between left and right wheel: \t%.3f \t(m)\n", _thread_length);
	printf("Radius of driving wheel: \t%.3f \t(m)\n", _driving_wheel_radius);
	printf("Radius of front wheel: \t%.3f \t(m)\n", _front_wheel_radius);
	printf("Radius of rear wheel: \t%.3f \t(m)\n", _rear_wheel_radius);
	printf("Angular wheel velocity limit: \t%.3f \t(rad/s)\n", _w_limit);
	printf("Angular wheel acceleration limit: \t%.3f \t(rad/s)\n", _wdot_limit);
	printf("Mean of left wheel velocity disturbance: \t%.3f \t\n", _w_l_noise_mean);
	printf("Standard deviation of left wheel velocity disturbance: \t%.3f \t\n", _w_l_noise_std);
	printf("Mean of right wheel velocity disturbance: \t%.3f \t\n", _w_r_noise_mean);
	printf("Standard deviation of right wheel velocity disturbance: \t%.3f \t\n", _w_r_noise_std);

	printf("\n\t== Coefficients of Slippage ==\n\n");
	printf("Mean of slip ratio: \t%.3f \t\n", _slip_ratio_mean);
	printf("Standard deviation of slip ratio: \t%.3f \t\n", _slip_ratio_std);
	printf("Coefficient of lateral resistance: \t%.3f \t\n", _mu_l);
	printf("Acceleration of gravity: \t%.3f \t(m/s^2)\n", _gravity_acc);
	printf("Minimum linear velocity to apply slippage: \t%.3f \t(m/s)\n", _v_threshod_to_apply_slippage);

	printf("\n");
	printf("\n");
	printf("***************************************\n");
}

} // namespace plugin
} // namespace rlab
