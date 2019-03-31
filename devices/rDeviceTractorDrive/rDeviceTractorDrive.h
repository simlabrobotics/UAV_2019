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

class rDeviceCombineDrive : public RD_DEVICE_CLASS(Base)
{
	RD_VERSION(1.0);
	RD_AUTHOR(SimLab);

public:
	RD_DECLARE_CTOR(CombineDrive);
	RD_DECLARE_DTOR(CombineDrive);

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

	// vehicle kinematics & dynamics
	float	_thread_length;			// distance between left and right track
	float	_front_wheel_radius;
	float	_rear_wheel_radius;
	float	_driving_wheel_radius;	// wheel radius of driving axis
	float	_mass;					// mass of vehicle
    float	_track_width;			// track width
	float	_track_length;			// track width
	
	// limits
	float	_w_limit;	// maximum wheel velocity (rad/sec)
	float	_wdot_limit;// maximum wheel acceleration (rad/sec^2)
	
	// desired(commanded) angular velocities of the wheels
	float	_w_l_des;
	float	_w_r_des;

	// current angular velocities of the wheels considering wheel velocity/acceleration limits w/o noise
	float	_w_l;
	float	_w_r;
	// current angular velocities of the wheels considering wheel velocity/acceleration limits with noise
	float	_w_l_w_noise;
	float	_w_r_w_noise;

	// current forward & angular velocity w/o slip (wrt vehicle coordinate frame)
	float	_v_wo_slip;
	float	_thdot_wo_slip;

	// current forward & angular velocity with slip (wrt vehicle coordinate frame)
	float	_v;
	float	_thdot;

	// current pose of tractor (wrt global coordinate frame)
	float	_gx;		// global position x
	float	_gy;		// global position y
	float	_gz;		// global position z
	float	_gth;		// yaw angle
	float	_pitch;		// pitch angle
	float	_roll;		// roll angle

	// current velocity (wrt global coordinate frame)
	float	_gxdot;
	float	_gydot;
	float	_gzdot;
	float	_gthdot;	// yaw rate
	float	_gthddot;	// yaw angular acceleration

	// initial transform of vehicle
	float	_r0[3];		// initial axis displacement
	float	_R0[9];		// initial axis rotation

	// current transformation of tractor
	float	_R[9]; 
	float	_r[3];

	// model parameters to determine the slip angle, slip ratio
	float	_slip_ratio_mean;	// norm of slip ratio
	float	_slip_ratio_std;	// standard deviation of slip ratio
	float	_i_r;				// slip ratio of right wheel
	float	_i_l;				// slip ratio of left wheel
	float	_mu_l;				// coefficient of lateral resistance
	float	_gravity_acc;		// acceleration of gravity
	float	_alpha;				// slip angle (rad)
	
	// minimum velocity to apply slippage
	float	_v_threshod_to_apply_slippage;

	// wheel position
	float	_front_lwheel_theta; // front left wheel position(rad)
	float	_front_rwheel_theta; // front right wheel position(rad)
	float	_rear_lwheel_theta; // rear left wheel position(rad)
	float	_rear_rwheel_theta; // rear right wheel position(rad)

	// noise parameter
	float	_w_l_noise_mean;
	float	_w_l_noise_std;
	float	_w_r_noise_mean;
	float	_w_r_noise_std;

	// indicators and coefficients to calculate indicators
	float	_F_r;			// tractive force of left track
	float	_F_l;			// tractive force of right track
	float	_z0;			// sinkage
	float	_Rc;			// motion resistance
	float	_K;				// deformation modulus
    float	_pi;			// internal shearing resistance
    float	_k_c;			// cohesive modulus of terrain deformation
    float	_k_pi;			// frictional modulus of terrain deformation
    float	_n;				// exponent of terrain deformation

	// height map
	HeightMap _hmap;
	int _hmap_normal_window_size;
};

} // namespace plugin
} // namespace rlab

#endif
