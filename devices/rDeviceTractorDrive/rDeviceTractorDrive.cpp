/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#include <rlab/utils/rSampleUtil.h>
#include "rDeviceTractorDrive.h"
#include "rDeviceTractorDriveCmd.h"
#include "UAV_def.h"

#define READ_PROP_BOOLEAN(prop_name, var_name, def_val)	\
	prop = getProperty(prop_name);						\
	if (prop) {											\
		if (_tcsicmp(prop, _T("true")) == 0 ||			\
			_tcsicmp(prop, _T("yes")) == 0)				\
			(##var_name) = true;						\
		else											\
			(##var_name) = false;						\
	}													\
	else (##var_name) = def_val;

#define READ_PROP_INT(prop_name, var_name, def_val)		\
	prop = getProperty(prop_name);						\
	if (prop) (##var_name) = _tstoi(prop);				\
	else (##var_name) = def_val;

#define READ_PROP_REAL(prop_name, var_name, def_val)	\
	prop = getProperty(prop_name);						\
	if (prop) (##var_name) = _tstof(prop);				\
	else (##var_name) = def_val;

#define READ_PROP_STR(prop_name, var_name, def_val)		\
	prop = getProperty(prop_name);						\
	if (prop) (##var_name) = prop;						\
	else (##var_name) = def_val;

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
	, _front_ltheta(0), _front_rtheta(0)
	, _rear_ltheta(0), _rear_rtheta(0)
	, _steer_ltheta(0), _steer_rtheta(0)
	, _v_des(0), _theta_des(0), _v(0), _theta(0)
	, _vx(0), _vy(0)
	, _x(0), _y(0), _z(0), _psi(0), _pitch(0), _roll(0)
	, _xdot(0), _ydot(0), _zdot(0)
	, _psidot(0), _psiddot(0)
	, _beta(0), _betadot(0)
	, _betaf(0), _betar(0)
	, _Ff(0), _Fr(0)
	, _hmap_normal_window_size(1)
	, _ax_slope(0), _ay_slope(0)
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
				_v_des = value[0];				// forward velocity
				_theta_des = value[1]*DEGREE;	// steer angle(degree)
			}
			_lock.unlock();
			return 2 * sizeof(float);
		}
	}

	case UAVDRV_DATAPORT_SET_PARAMETER:
	{
		if (len >= 4 * sizeof(float))
		{
			_lock.lock();
			{
				int *ivalue = (int*)buffer;
				float *fvalue = (float*)buffer;
				switch (ivalue[0])
				{
				case PTYPE_C_vehicle_mass:
					if (fvalue[1] > 0.0f) _m = fvalue[1];
					break;
				case PTYPE_C_vehicle_moment_of_inertia_at_COG:
					if (fvalue[1] > 0.0f) _Iz = fvalue[1];
					break;
				case PTYPE_C_gravitational_acceleration:
					if (fvalue[1] > 0.0f) _g = fvalue[1];
					break;
				case PTYPE_C_slip_ratio_mean_and_std:
				case PTYPE_C_maximum_velocity:
				case PTYPE_C_maximum_acceleration:
				case PTYPE_C_maximum_steer_angle:
				case PTYPE_C_maximum_steer_velocity:
				case PTYPE_C_steer_ratio:
				case PTYPE_C_velocity_noise_mean_and_std:
				case PTYPE_C_velocity_threshold_to_apply_slippage:
				case PTYPE_C_Cf:
					if (fvalue[1] > 0.0f) _Cf = fvalue[1];
					break;
				case PTYPE_C_Cr:
					if (fvalue[1] > 0.0f) _Cr = fvalue[1];
					break;
				case PTYPE_C_Lf:
					if (fvalue[1] > 0.0f) _Lf = fvalue[1];
					break;
				case PTYPE_C_Lr:
					if (fvalue[1] > 0.0f) _Lr = fvalue[1];
					break;
				case PTYPE_C_velocity_threshold_to_apply_slippage_due_to_slope:
				case PTYPE_C_tire_radius:
				case PTYPE_C_tire_contact_length:
				case PTYPE_C_tire_contact_width:
				case PTYPE_C_soil_adhesiveness:
				case PTYPE_C_internal_shearing_resistance:
				case PTYPE_C_shear_modulus_of_elasticity:
				case PTYPE_C_cohesive_modulus_of_terrain_deformation:
				case PTYPE_C_frictional_modulus_of_terrain_deformation:
				case PTYPE_C_exponent_of_terrain_deformation:
				case PTYPE_C_slope_slip_ratio_mean_and_std:
				case PTYPE_C_ratio_of_terrain_deformation:
					break;
				}
			}
			_lock.unlock();
			return 4 * sizeof(float);
		}
	}
	break;

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

	case UAVDRV_MONITORPORT_SLIP_3D:
	{
		if (len >= 2 * sizeof(float))
		{
			_lock.lock();
			{
				float* value = (float*)buffer;
				value[0] = _ax_slope;
				value[1] = _ay_slope;
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
		//
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
		//
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
		//
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
		if (_v >= _v_threshod_to_apply_slippage_f ||
			_v <= _v_threshod_to_apply_slippage_b)
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


		/////////////////////////////////////////////////////////////////////
		// calcurate slippage due to soid deformation wrt vehicle pose,
		// and apply it to update vehicle position
		//
		if (_v >= _v_threshod_to_apply_slopeslip_f ||
			_v <= _v_threshod_to_apply_slopeslip_b)
		{
			float k_sign = (sample_uniform01() > 0.5 ? 1 : -1);
			float slope_slip_ratio = sample_quasi_normal(_s_mean, _s_std);
			float tire_b;
			std::vector<float> a_slope;
			std::vector<float> slope_angles;
			slope_angles.push_back(_pitch); // pitch angle introduce forward acceleration
			slope_angles.push_back(-_roll); // roll angle introduce lateral acceleration in opposite direction
			for_each(slope_angles.begin(), slope_angles.end(), [&](float slope) {
				tire_b = (_m * _g * cos(slope) / (4.0 * _tire_b * _tire_w * (_k_c / _tire_w + _k_pi)));
				tire_b = pow(tire_b, (1.0 / _n));
				tire_b = pow((_tire_r - tire_b), 2.0);
				tire_b = sqrt(_tire_r * _tire_r - tire_b);

				float a = _g * sin(slope) - (tire_b * _tire_w / _m) * ((_c + _m * _g * cos(slope) * tan(_pi)) * (1 + k_sign * exp(-slope_slip_ratio * _tire_w / _K)));
				if ((slope * a) <= 0) a = 0; // sign of 'slope' and 'a' cannot be opposite.
				a_slope.push_back(a);
			});

			_ax_slope = a_slope[0];
			_ay_slope = a_slope[1];

			float vx_slope = _ax_slope * _dT;
			float vy_slope = _ay_slope * _dT;
			float xdot_slope = vx_slope*cos(_psi) - vy_slope*sin(_psi);
			float ydot_slope = vx_slope*sin(_psi) + vy_slope*cos(_psi);
			_x += xdot_slope*_dT;
			_y += ydot_slope*_dT;
		}


		/////////////////////////////////////////////////////////////////////
		// new vehicle(tractor) transformation
		//
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

		/////////////////////////////////////////////////////////////////////
		// visualizing rotating wheel and steer angles
		//
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

void rDeviceTractorDrive::ComputeIndicators()
{

}

// initialize device parameters
void rDeviceTractorDrive::InitParams()
{
	// initialize device parameters
	const TCHAR* prop;

	// 
	READ_PROP_REAL(_T("distance_btw_axes"), _distance_btw_axes, 1.0f);

	// front wheels 
	_front_lwheel = INVALID_RID;
	_front_rwheel = INVALID_RID;
	READ_PROP_REAL(_T("front_radius"), _front_wheel_radius, 1.0f);
	prop = getProperty(_T("front_lwheel"));
	if (prop)
		_front_lwheel = _rdc.m_deviceAPI->getDeviceID(_rdc.m_robotname, prop);
	prop = getProperty(_T("front_rwheel"));
	if (prop)
		_front_rwheel = _rdc.m_deviceAPI->getDeviceID(_rdc.m_robotname, prop);

	// rear wheels
	_rear_lwheel = INVALID_RID;
	_rear_rwheel = INVALID_RID;
	READ_PROP_REAL(_T("rear_radius"), _rear_wheel_radius, 1.0f);
	prop = getProperty(_T("rear_lwheel"));
	if (prop)
		_rear_lwheel = _rdc.m_deviceAPI->getDeviceID(_rdc.m_robotname, prop);
	prop = getProperty(_T("rear_rwheel"));
	if (prop)
		_rear_rwheel = _rdc.m_deviceAPI->getDeviceID(_rdc.m_robotname, prop);

	// stearing
	_steer_lwheel = INVALID_RID;
	_steer_rwheel = INVALID_RID;
	prop = getProperty(_T("steer_lwheel"));
	if (prop)
		_steer_lwheel = _rdc.m_deviceAPI->getDeviceID(_rdc.m_robotname, prop);
	prop = getProperty(_T("steer_rwheel"));
	if (prop)
		_steer_rwheel = _rdc.m_deviceAPI->getDeviceID(_rdc.m_robotname, prop);
	
	// limits (negative value means it is not active.)
	READ_PROP_REAL(_T("steer_limit"), _steer_limit, -1.0f);
	_steer_limit *= DEGREE;
	READ_PROP_REAL(_T("steer_velocity_limit"), _steer_velocity_limit, -1.0f);
	_steer_velocity_limit *= DEGREE;

	READ_PROP_REAL(_T("velocity_limit"), _velocity_limit, -1.0f);
	READ_PROP_REAL(_T("acceleration_limit"), _acceleration_limit, -1.0f);

	//
	READ_PROP_REAL(_T("Cf"), _Cf, -1.0f);
	READ_PROP_REAL(_T("Cr"), _Cr, -1.0f);
	READ_PROP_REAL(_T("Iz"), _Iz, -1.0f);
	READ_PROP_REAL(_T("Lf"), _Lf, -1.0f);
	READ_PROP_REAL(_T("Lr"), _Lr, -1.0f);
	
	READ_PROP_REAL(_T("slip_ratio"), _slip_ratio, 0.0f);
	_slip_ratio = (float)RD_BOUND(_slip_ratio, 0, 1.0);

	READ_PROP_REAL(_T("velocity_threshold_to_apply_slippage_f"), _v_threshod_to_apply_slippage_f, 0.3f);
	_v_threshod_to_apply_slippage_f = (float)RD_LBOUND(_v_threshod_to_apply_slippage_f, 1.0e-4);
	READ_PROP_REAL(_T("velocity_threshold_to_apply_slippage_b"), _v_threshod_to_apply_slippage_b, -0.3f);
	_v_threshod_to_apply_slippage_b = (float)RD_UBOUND(_v_threshod_to_apply_slippage_b, -1.0e-4);

	// tire dynamics
	READ_PROP_REAL(_T("tire_radius(R_tire|m)"), _tire_r, 1.0f);
	READ_PROP_REAL(_T("tire_contact_length(b_init|m)"), _tire_b, 1.0f);
	READ_PROP_REAL(_T("tire_contact_width(w|m)"), _tire_w, 1.0f);
	
	// dynamics
	READ_PROP_REAL(_T("mass(m|kg)"), _m, 1.0f);
	READ_PROP_REAL(_T("gravitational_acc(g|m/s^2)"), _g, 1.0f);
	
	// soil deformation
	READ_PROP_REAL(_T("soil_adhesiveness(c|Pa)"), _c, 1.0f);
	READ_PROP_REAL(_T("internal_shearing_resistance(pi|deg)"), _pi, 1.0f);
	_pi *= DEGREE;
	READ_PROP_REAL(_T("shear_modulus_of_elasticity(K|m)"), _K, 1.0f);
	READ_PROP_REAL(_T("cohesive_modulus_of_terrain_deformation(k_c|Pa)"), _k_c, 1.0f);
	READ_PROP_REAL(_T("frictional_modulus_of_terrain_deformation(k_pi|Pa)"), _k_pi, 1.0f);
	READ_PROP_REAL(_T("exponent_of_terrain_deformation(n)"), _n, 1.0f);
	READ_PROP_REAL(_T("ratio_of_terrain_deformation(K_sign)"), _K_sign, 0.0f); 
	READ_PROP_REAL(_T("slope_slip_ratio_mean(s_mean)"), _s_mean, 0.0f);
	READ_PROP_REAL(_T("slope_slip_ratio_std(s_std)"), _s_std, 0.0f);
	READ_PROP_REAL(_T("velocity_threshold_to_apply_slopeslip_f(v_th_slopeslip_f|m/s)"), _v_threshod_to_apply_slopeslip_f, 0.0f);
	_v_threshod_to_apply_slopeslip_f = (float)RD_LBOUND(_v_threshod_to_apply_slopeslip_f, 1.0e-4);
	READ_PROP_REAL(_T("velocity_threshold_to_apply_slopeslip_b(v_th_slopeslip_b|m/s)"), _v_threshod_to_apply_slopeslip_b, 0.0f);
	_v_threshod_to_apply_slopeslip_b = (float)RD_UBOUND(_v_threshod_to_apply_slopeslip_b, -1.0e-4);
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
	printf("\n\t== PARAMETERS OF VEHICLE KINEMATICS ==\n\n");
	printf("Distance between front and rear axis: \t%.3f \t(m)\n", _distance_btw_axes);
	printf("Radius of front wheel: \t%.3f \t(m)\n", _front_wheel_radius);
	printf("Radius of rear wheel: \t%.3f \t(m)\n", _rear_wheel_radius);
	printf("Steering angle limit: \t%.3f \t(degree)\n", _steer_limit*RADIAN);
	printf("Steering velocity limit: \t%.3f \t(degree/s)\n", _steer_velocity_limit*RADIAN);
	printf("Longitudinal velocity limit: \t%.3f \t(m/s)\n", _velocity_limit);
	printf("Longitudinal acceleration limit: \t%.3f \t(m/s^2)\n", _acceleration_limit);
	printf("\n\t== PARAMETERS OF VEHICLE DYNAMICS ==\n\n");
	printf("Cornering stiffness of front wheels(Cf): \t%.3f \t(N/rad)\n", _Cf);
	printf("Cornering stiffness of rear wheels(Cr): \t%.3f \t(N/rad)\n", _Cr);
	printf("Mass of the tractor(m): \t%.3f \t(kg)\n", _m);
	printf("Turning inertia WRT tractor COG(Iz): \t%.3f \t(kg m^2)\n", _Iz);
	printf("Distance from front wheel to tractor COG(Lf): \t%.3f \t(m)\n", _Lf);
	printf("Distance from rear wheel to tractor COG(Lr): \t%.3f \t(m)\n", _Lr);
	printf("\n\t== PARAMETERS OF TIRE DYNAMICS ==\n\n");
	printf("Tire radius(R_tire): \t%.3f \t(m)\n", _tire_r);
	printf("Tire contact length(b): \t%.3f \t(m)\n", _tire_b);
	printf("Tire contact width(w): \t%.3f \t(m)\n", _tire_w);
	printf("Gravitational acceleration(g): \t%.3f \t(m/s^2)\n", _g);
	printf("\n\t== PARAMETERS OF SOIL DEFORMATION ==\n\n");
	printf("Soil adhesiveness(c): \t%.3f \t(Pa)\n", _c);
	printf("Inertial friction angle(pi): \t%.3f \t(deg)\n", _pi*RADIAN);
	printf("Shear modulus of elasticity(K): \t%.3f \t(m)\n", _K);
	printf("Cohesive modulus of terrain deformation(k_c): \t%.3f \t(Pa)\n", _k_c);
	printf("Frictional modulus of terrain deformation(k_pi): \t%.3f \t(Pa)\n", _k_pi);
	printf("Exponent of terrain deformation(n): \t%.3f \t\n", _n);
	printf("Ratio of terrain deformation(K_sign): \t%.3f\n", _K_sign); 
	printf("Slope slip ratio(s_mean, s_std): \t%.3f \t%0.3f\n", _s_mean, _s_std);
	printf("\n\t== OTHER PARAMETERS ==\n\n");
	printf("Forward/backward velocity threshold to apply slippage: \t%.3f \t(m/s)\t%0.3f \t(m/s)\n", _v_threshod_to_apply_slippage_f, _v_threshod_to_apply_slippage_b);
	printf("Forward/backward velocity threshold to apply slippage due to slope: \t%.3f \t(m/s)\t%0.3f \t(m/s)\n", _v_threshod_to_apply_slopeslip_f, _v_threshod_to_apply_slopeslip_b);
	printf("\n");
	printf("\n");
	printf("***************************************\n");
}

} // namespace plugin
} // namespace rlab
