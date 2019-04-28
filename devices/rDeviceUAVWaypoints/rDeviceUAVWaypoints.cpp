/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#include <rlab/utils/rSampleUtil.h>
#include <rlab/utils/rParseUtil.h>
#include "rDeviceUAVWaypoints.h"

namespace rlab {
namespace plugin {

////////////////////////////////////////////////////////////////
// implementation of rDeviceUAVWaypoints
//
RD_IMPLE_FACTORY(UAVWaypoints)

rDeviceUAVWaypoints::rDeviceUAVWaypoints()
: _waypoint_added(0)
{
}

rDeviceUAVWaypoints::~rDeviceUAVWaypoints()
{
}

void rDeviceUAVWaypoints::onCreate(const rDeviceContext& rdc)
{
	RD_DEVICE_CLASS(Base)::onCreate(rdc);
}

void rDeviceUAVWaypoints::onInit()
{
	RD_DEVICE_CLASS(Base)::onInit();

	_waypoint_added = 0;
}

void rDeviceUAVWaypoints::onTerminate()
{
	RD_DEVICE_CLASS(Base)::onTerminate();
}

int rDeviceUAVWaypoints::readDeviceValue(void* buffer, int len, int port)
{
	int read = 0;

	_lock.lock();
	{
		if (_waypoint_added != 0 &&
			len >= _waypoints.size() * sizeof(WAYPOINT))
		{
			*((int*)buffer) = (int)_waypoints.size();
			WAYPOINT* pwp = (WAYPOINT*)((int*)buffer + 1);
			for (std::list<WAYPOINT>::iterator itr = _waypoints.begin(); itr != _waypoints.end(); itr++)
			{
				*pwp = *itr;
				pwp++;
			}
			read = sizeof(int) + _waypoints.size() * sizeof(WAYPOINT);
			_waypoint_added = 0;
		}
	}
	_lock.unlock();

	return read;
}

int rDeviceUAVWaypoints::writeDeviceValue(void* buffer, int len, int port)
{
	if (len == sizeof(WAYPOINT))
	{
		_lock.lock();
		{
			WAYPOINT wp = *(WAYPOINT*)buffer;
			if (wp.index == 1)
				Reset();

			_waypoints.push_front(wp);
			_waypoint_added = 1;
		}
		_lock.unlock();
		return sizeof(WAYPOINT);
	}
	else
		return 0;
}

int rDeviceUAVWaypoints::monitorDeviceValue(void* buffer, int len, int port)
{
	return 0;
}

void rDeviceUAVWaypoints::importDevice(rTime time, void* mem)
{
}

void rDeviceUAVWaypoints::Reset()
{
	_waypoints.clear();
	_waypoint_added = 0;
}

} // namespace plugin
} // namespace rlab
