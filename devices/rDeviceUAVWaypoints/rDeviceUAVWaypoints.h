/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */

#ifndef __RDEVICEUAVWAYPOINTS_H__
#define __RDEVICEUAVWAYPOINTS_H__

#include <rlab/device/rDeviceBase.h>
#include <rlab/math/rmath.h>
#include <list>
#include "UAV_def.h"

namespace rlab {
namespace plugin {

class rDeviceUAVWaypoints : public RD_DEVICE_CLASS(Base)
{
	RD_VERSION(1.0);
	RD_AUTHOR(SimLab);

public:
	RD_DECLARE_CTOR(UAVWaypoints);
	RD_DECLARE_DTOR(UAVWaypoints);

public:
	RD_DECLARE_createDevice;
	RD_DECLARE_initDevice;
	RD_DECLARE_terminateDevice;
	RD_DECLARE_readDeviceValue;
	RD_DECLARE_writeDeviceValue;
	RD_DECLARE_monitorDeviceValue;
	RD_DECLARE_importDevice;

	void InitParams();
	void PrintParams();
	void InitHeightMap();

private:
	std::list<WAYPOINT>		_waypoints;
	int						_waypoint_added;

private:
	void					Reset();
};

} // namespace plugin
} // namespace rlab

#endif // __RDEVICEUAVWAYPOINTS_H__
