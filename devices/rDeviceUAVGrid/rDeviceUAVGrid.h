/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */

#ifndef __RDEVICEUAVGRID_H__
#define __RDEVICEUAVGRID_H__

#include <rlab/device/rDeviceBase.h>
#include <rlab/math/rmath.h>

#define GRIDDATA 10

namespace rlab {
namespace plugin {

class rDeviceUAVGrid : public RD_DEVICE_CLASS(Base)
{
	RD_VERSION(1.0);
	RD_AUTHOR(SimLab);

public:
	RD_DECLARE_CTOR(UAVGrid);
	RD_DECLARE_DTOR(UAVGrid);

public:
	RD_DECLARE_createDevice;
	RD_DECLARE_initDevice;
	RD_DECLARE_terminateDevice;
	RD_DECLARE_readDeviceValue;
	RD_DECLARE_writeDeviceValue;
	RD_DECLARE_monitorDeviceValue;
	RD_DECLARE_importDevice;
	RD_DECLARE_command;

private:
	string_type				_tool_robotName;
	string_type				_tool_bodyName;
	string_type				_tool_deviceName;
	rID						_tool_bodyID;
	rID						_tool_deviceID;
	HTransform				_tool_bodyHT;
	HTransform				_tool_offset;
	float					_tool_width;
	enum eTOOL_TYPE {
		TOOL_NONE,
		TOOL_LINE,
	};
	eTOOL_TYPE				_tool_type;
	bool					_tool_activated;

	int						_rows;
	int						_cols;
	int						_cell_count;
	int						_cell_count_occupied;
	float					_cell_size;

	unsigned char			*_grid;

	int						_read_offset;
	int						_read_index;
	int						_read_size_max;

private:
	void					Reset();
};

} // namespace plugin
} // namespace rlab

#endif // __RDEVICEUAVGRID_H__
