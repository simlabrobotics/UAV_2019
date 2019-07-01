/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */

#ifndef __rDeviceUAVPTO_H__
#define __rDeviceUAVPTO_H__

#include <rlab/device/rDeviceBase.h>

namespace rlab {
namespace plugin {

class rDeviceUAVPTO : public RD_DEVICE_CLASS(Base)
{
	RD_VERSION(1.1);
	RD_AUTHOR(SimLab);

public:
	RD_DECLARE_CTOR(UAVPTO);
	RD_DECLARE_DTOR(UAVPTO);
public:
	RD_DECLARE_createDevice;
	RD_DECLARE_initDevice;
	RD_DECLARE_terminateDevice;
	RD_DECLARE_writeDeviceValue;
	RD_DECLARE_readDeviceValue;
	RD_DECLARE_updateWriteValue;

private:
	rID _jointid;
	float _period;
	float _q[RD_DOF_MAX];			 // current joint position
	float _q_des[RD_DOF_MAX];		 // desired joint position
	float _q_offset[RD_DOF_MAX];     // joint position offset
	float _q_limit_U[RD_DOF_MAX];	 // upper joint limit (since kinematic motor v1.1)	
	float _q_limit_L[RD_DOF_MAX];	 // lower joint limit (since kinematic motor v1.1)
	float _qdot_limit[RD_DOF_MAX];   // joint velocity limit
	float _q_prev[RD_DOF_MAX];       // previous joint position
	float _reduction[RD_DOF_MAX];	 // gear reduction ratio (since kinematic motor v1.2)
	float _reduction_inv[RD_DOF_MAX];// inverse of gear reduction ratio (since kinematic motor v1.2)
	unsigned short _dim;			 // joint dimension
	bool _update_adj_kinematics;	 // whether to update adjacent kinematics
	float _q_PTO_active[RD_DOF_MAX];	 // joint angle when PTO is active
	float _q_PTO_deactive[RD_DOF_MAX];	 // joint angle when PTO is deactive

private:
	void _isActivated(char* state);
};

} // namespace plugin
} // namespace rlab

#endif // __rDeviceUAVPTO_H__
