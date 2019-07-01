/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#include "rDeviceUAVPTO.h"
#include "rlab/utils/rParseUtil.h"
#include <float.h>


namespace rlab {
namespace plugin {

RD_IMPLE_FACTORY(UAVPTO)

rDeviceUAVPTO::rDeviceUAVPTO() 
{
}

rDeviceUAVPTO::~rDeviceUAVPTO()
{
}

void rDeviceUAVPTO::onCreate(const rDeviceContext& rdc)
{
	RD_DEVICE_CLASS(Base)::onCreate(rdc);

	userType = RD_TYPE_KINEMATIC_MOTOR;

	const TCHAR* prop = NULL;
	
	for (int i=0; i<RD_DOF_MAX; i++)
	{
		_q[i] = 0.0f;
		_q_des[i] = 0.0f;
		_q_limit_L[i] = -FLT_MAX;
		_q_limit_U[i] =  FLT_MAX;
		_qdot_limit[i] = FLT_MAX;
		_q_prev[i] = _q[i];
		_q_offset[i] = 0.0f;
		_reduction[i] =  1.0f;
		_reduction_inv[i] = 1.0f;
		_q_PTO_active[i] = 0.0f;
		_q_PTO_deactive[i] = 0.0f;
	}
	if (NULL != (prop = getProperty(_T("joint_limit_lower"))))
	{
		parse_vector(_q_limit_L, RD_DOF_MAX, prop);
		for (int i=0; i<RD_DOF_MAX; i++)
			_q_limit_L[i] = (_q_limit_L[i] == -FLT_MAX ? _q_limit_L[0] : _q_limit_L[i]*DEGREE);
	}
	if (NULL != (prop = getProperty(_T("joint_limit_upper"))))
	{
		parse_vector(_q_limit_U, RD_DOF_MAX, prop);
		for (int i=0; i<RD_DOF_MAX; i++)
			_q_limit_U[i] = (_q_limit_U[i] == FLT_MAX ? _q_limit_U[0] : _q_limit_U[i]*DEGREE);
	}
	if (NULL != (prop = getProperty(_T("velocity_limit"))))
	{
		parse_vector(_qdot_limit, RD_DOF_MAX, prop);
		for (int i=0; i<RD_DOF_MAX; i++)
			_qdot_limit[i] = (_qdot_limit[i] == FLT_MAX ? _qdot_limit[0] : _qdot_limit[i]*DEGREE);
	}
	if (NULL != (prop = getProperty(_T("reduction"))))
	{
		parse_vector(_reduction, RD_DOF_MAX, prop);
		for (int i=0; i<RD_DOF_MAX; i++) {
			_reduction[i] = (_reduction[i] == FLT_MAX ? _reduction[0] : _reduction[i]);
			_reduction_inv[i] = 1.0f / _reduction[i];
		}
	}

	_update_adj_kinematics = false;

	if (NULL != (prop = getProperty(_T("update_adj_kinematics"))))
	{
		if (0 == _tcsicmp(prop, _T("true")))
			_update_adj_kinematics = true;
	}

	if (NULL != (prop = getProperty(_T("offset"))))
	{
		for (int i=0; i<RD_DOF_MAX; i++)
			_q_offset[i] = FLT_MAX;
		parse_vector(_q_offset, RD_DOF_MAX, prop);
		for (int i=0; i<RD_DOF_MAX; i++)
			_q_offset[i] = (_q_offset[i] == FLT_MAX ? _q_offset[0] : _q_offset[i]*DEGREE);
		for (int i=0; i<RD_DOF_MAX; i++)
			if (_q_offset[i] == FLT_MAX) _q_offset[i] = 0.0f;
	}

	if (NULL != (prop = getProperty(_T("PTO_active_angle"))))
	{
		parse_vector(_q_PTO_active, RD_DOF_MAX, prop);
		for (int i = 0; i<RD_DOF_MAX; i++)
			_q_PTO_active[i] = (_q_PTO_active[i] == -FLT_MAX ? _q_PTO_active[0] : _q_PTO_active[i] * DEGREE);
	}
	if (NULL != (prop = getProperty(_T("PTO_deactive_angle"))))
	{
		parse_vector(_q_PTO_deactive, RD_DOF_MAX, prop);
		for (int i = 0; i<RD_DOF_MAX; i++)
			_q_PTO_deactive[i] = (_q_PTO_deactive[i] == -FLT_MAX ? _q_PTO_deactive[0] : _q_PTO_deactive[i] * DEGREE);
	}

	_period = 0.0f;
}

void rDeviceUAVPTO::onInit()
{
	RD_DEVICE_CLASS(Base)::onInit();

	_jointid = _rdc.m_deviceAPI->getJointID(_rdc.m_robotname, _rdc.m_nodename);
	if (_jointid == INVALID_RID)
	{
		TCHAR errMsg[512];
#ifndef __GNUC__
		_stprintf_s(errMsg, 512, _T("Error occurs when creating kinematic motor\n*system name:%s\n*node name:%s"), _rdc.m_robotname, _rdc.m_nodename);
#else
		_stprintf(errMsg, 512, _T("Error occurs when creating kinematic motor\n*system name:%s\n*node name:%s"), _rdc.m_robotname, _rdc.m_nodename);
#endif
		throw rDevicePluginException(errMsg);
	}
	_dim = _rdc.m_deviceAPI->getJointDOF(_jointid);

	_rdc.m_deviceAPI->getJointPosition(_jointid, _q);	

	rID sysid = _rdc.m_deviceAPI->getSystemID(_rdc.m_robotname);
	bool isStatic;
	_rdc.m_deviceAPI->isSystemStatic(sysid, &isStatic);
	if (!isStatic)
		_rdc.m_deviceAPI->setKinematicJoint(sysid, _jointid);
}

void rDeviceUAVPTO::onTerminate()
{
	RD_DEVICE_CLASS(Base)::onTerminate();
}

int	rDeviceUAVPTO::readDeviceValue(void* buffer, int len, int port)
{
	if (len >= (int)_dim * (int)sizeof(float))
	{
		_lock.lock();
		memcpy(buffer, _q, sizeof(float)*_dim);
		_lock.unlock();
		return sizeof(float) * _dim;
	}
	else if (len >= (int)_dim * (int)sizeof(char))
	{
		_lock.lock();
		char state[RD_DOF_MAX];
		_isActivated(state);
		memcpy(buffer, state, sizeof(char)*_dim);
		_lock.unlock();
		return sizeof(char) * _dim;
	}
	else
		return 0;
}

int rDeviceUAVPTO::writeDeviceValue(void* buffer, int len, int port)
{
	if (len >= _dim * sizeof(float))
	{
		float* value = (float*)buffer;
		_lock.lock();
		{
			for (int i = 0; i < _dim; i++)
				_q_des[i] = value[i]*_reduction_inv[i];
		}
		_lock.unlock();
		return (_dim * sizeof(float));
	}
	else if (len >= _dim * sizeof(char))
	{
		// 0: deactivate PTO, 1: activate PTO
		char* value = (char*)buffer;
		_lock.lock();
		{
			for (int i = 0; i < _dim; i++)
				_q_des[i] = (value[i] == 0 ? _q_PTO_deactive[i] : _q_PTO_active[i])*_reduction_inv[i];
		}
		_lock.unlock();
		return (_dim * sizeof(char));
	}
	else
		return 0;
}

void rDeviceUAVPTO::updateWriteValue(rTime time)
{
	if (_period < 1.0e-6 && time > 1.0e-6) {
		_period = time;
		for (int i=0; i<_dim; i++) {
			_qdot_limit[i] *= _period;
		}
	}

	_lock.lock();
	{
		float q[RD_DOF_MAX];
		float q_des[RD_DOF_MAX];
		for (int i=0; i<_dim; i++) {
			if (fabs(_q_des[i]-_q_prev[i]) >= _qdot_limit[i])
			{
				if (_q_des[i] > _q_prev[i])
					q_des[i] = _q_prev[i] + _qdot_limit[i];
				else
					q_des[i] = _q_prev[i] - _qdot_limit[i];
			}
			else
			{
				q_des[i] = _q_des[i];
			}
			_q[i] = RD_BOUND(q_des[i], _q_limit_L[i], _q_limit_U[i]);
			q[i] = _q[i] + _q_offset[i];
			_q_prev[i] = _q[i];
		}
		if (_update_adj_kinematics)
			_rdc.m_deviceAPI->setJointPosition(_jointid, q, false);
		else
			_rdc.m_deviceAPI->setJointPositionKinematic(_jointid, q);
	}
	_lock.unlock();
}

void rDeviceUAVPTO::_isActivated(char* state)
{
	float q_threshold;
	for (int i = 0; i < _dim; i++)
	{
		q_threshold = abs(_q_PTO_active[i] - _q_PTO_deactive[i]) * 0.05; // +/-5%
		state[i] = ((_q[i] > (_q_PTO_active[i] - q_threshold) && _q[i] < (_q_PTO_active[i] + q_threshold)) ? 1 : 0);
	}
}

} // namespace plugin
} // namespace rlab
