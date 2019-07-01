/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */

#ifndef __RDEVICECOMBINEDRIVE_H__
#define __RDEVICECOMBINEDRIVE_H__

#include <rlab/device/rDeviceBase.h>
#include <rlab/math/rmath.h>
#include "HeightMap.h"

namespace rlab {
namespace plugin {

class rDeviceTractorDrive : public RD_DEVICE_CLASS(Base)
{
	RD_VERSION(1.0);
	RD_AUTHOR(SimLab);

public:
	RD_DECLARE_CTOR(TractorDrive);
	RD_DECLARE_DTOR(TractorDrive);

public:
	RD_DECLARE_createDevice;
	RD_DECLARE_initDevice;
	RD_DECLARE_terminateDevice;
	RD_DECLARE_writeDeviceValue;
	RD_DECLARE_monitorDeviceValue;
	RD_DECLARE_exportDevice;

	void InitParams();
	void PrintParams();
	void InitHeightMap();
	void ComputeIndicators();

private:
	// base body id
	rID		_nodeid;

	// simulation time
	rTime	_prevTime;
	float	_dT;
	
	// wheel motors
	rID		_front_lwheel;
	rID		_front_rwheel;
	rID		_rear_lwheel;
	rID		_rear_rwheel;
	rID		_steer_lwheel;
	rID		_steer_rwheel;

	// tractor kinematics
	float	_distance_btw_axes;
	float	_front_wheel_radius;
	float	_rear_wheel_radius;

	// current/desired wheel/steer position
	float	_front_ltheta;
	float	_front_rtheta;
	float	_rear_ltheta;
	float	_rear_rtheta;
	float	_steer_ltheta;
	float	_steer_rtheta;

	// limits
	float	_steer_limit;
	float	_steer_velocity_limit;
	float	_velocity_limit;
	float	_acceleration_limit;

	// desired(commanded) forward velocity and steer angle
	float	_v_des;
	float	_theta_des;

	// current forward velocity and steer angle (wrt vehicle coordinate frame)
	float	_v;
	float	_theta;

	// current forward and lateral velocity considering slip (wrt vehicle coordinate frame)
	float	_vx; // forward(longitudinal) speed
	float	_vy; // lateral speed

				 // current pose of tractor (wrt global coordinate frame)
	float	_psi;   // yaw angle
	float	_x;	    // global position x
	float	_y;	    // global position y
	float	_z;	    // global position z
	float	_pitch; // pitch angle
	float	_roll;  // roll angle

					// initial transform of tractor
	float	_r0[3];		// initial axis displacement
	float	_R0[9];		// initial axis rotation

						// current transformation of tractor
	float	_R[9];
	float	_r[3];

	// model parameters to determine the slip angle, yaw angle, yaw rate
	float	_Cf;		// cornering stiffness of front wheels (N/rad)
	float	_Cr;		// cornering stiffness of rear wheels (N/rad)
	float	_m;			// mass of the tractor (kg)
	float	_Iz;		// turning inertia WRT tractor COG (kg m^2)
	float	_Lf;		// distance from front wheel to tractor COG (m)
	float	_Lr;		// distance from rear wheel to tractor COG (m)

	float	_beta;		// slip angle of front and rear wheels
	float	_betadot;	// slip rate
	float	_betaf;		// slip angle of front wheel
	float	_betar;		// slip angle of rear wheel
	float	_Ff;		// lateral force at front wheel
	float	_Fr;		// lateral force at rear wheel

						// current velocity (wrt global coordinate frame)
	float	_xdot;
	float	_ydot;
	float	_zdot;
	float	_psidot;	// yaw rate
	float	_psiddot;	// yaw angular acceleration

	float	_slip_ratio;

	// forward/backward velocity threshold to apply extended bicycle model with lateral vehicle dynamics
	float	_v_threshod_to_apply_slippage_f;
	float	_v_threshod_to_apply_slippage_b;

	// model parameters to calculate slippage wrt soil deformation (2019/06)
	float	_tire_r;	// radius of tire
	float	_tire_b;	// initial contact length of tire
	float	_tire_w;	// contact width of tire
	float	_g;			// gravitational acceleration
	float	_c;			// soil adhesiveness
	float	_pi;		// internal shearing resistance
	float	_K;			// shear modulus of elasticity
	float	_k_c;		// cohesive modulus of terrain deformation
	float	_k_pi;		// frictional modulus of terrain deformation
	float	_K_sign;	// ratio of terrain deformation
	float	_n;			// exponent of terrain deformation
	float	_s_mean;	// slope slip ratio (mean)
	float	_s_std;		// slope slip ratio (standard deviation)
	float	_ax_slope;	// forward acceleration due to slope and soil deformation
	float	_ay_slope;	// lateral acceleration due to slope and soil deformation
	float	_v_threshod_to_apply_slopeslip_f; // forward velocity threshold to apply slippage due to slope
	float	_v_threshod_to_apply_slopeslip_b; // backward velocity threshold to apply slippage due to slope

	// height map (to calculate vehicle pose wrt terrain)
	HeightMap _hmap;
	int _hmap_normal_window_size;
};

} // namespace plugin
} // namespace rlab

#endif
