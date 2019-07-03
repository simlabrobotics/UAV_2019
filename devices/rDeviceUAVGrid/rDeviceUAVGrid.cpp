/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#include <rlab/utils/rSampleUtil.h>
#include <rlab/utils/rParseUtil.h>
#include "rDeviceUAVGrid.h"
#include "UAV_cmd.h"
#include "HeightMap.h"
//
// MAX_SINGLE_DEVICE_BUFFER_SIZE_TO_SEND is definded in rxsdk(rxDeviceManager.cpp) as...
//
#define MAX_SINGLE_DEVICE_BUFFER_SIZE_TO_SEND	(32*1024)  
//
// One readDeviceValue() function call should not return data greater than this.
// If so, assertion will fail in debug build or the device value will not be deliverd to
// rgd(graphics plug-in).
// To avoid this limit, one readDeviceValue() function call will split the whole data and 
// return only the partial data buffer.
// If the user want to read whole device buffer, readDeviceValue() should be called multiple times
// until it gets whole device buffer.
//
#define RDEVICEGRID_READSIZE_MAX 32000 


namespace rlab {
namespace plugin {

////////////////////////////////////////////////////////////////
// implementation of rDeviceUAVGrid
//
RD_IMPLE_FACTORY(UAVGrid)

rDeviceUAVGrid::rDeviceUAVGrid() 
: _tool_robotName(_T(""))
, _tool_bodyName(_T(""))
, _tool_deviceName(_T(""))
, _tool_deviceName_PTO(_T(""))
, _tool_bodyID(INVALID_RID)
, _tool_deviceID(INVALID_RID)
, _tool_deviceID_PTO(INVALID_RID)
, _tool_width(1.0f)
, _tool_type(TOOL_LINE)
, _tool_activated(0)
, _rows(1)
, _cols(1)
, _cell_count(1)
, _cell_count_wa(0)
, _cell_count_occupied(0)
, _cell_size(0.1f)
, _grid(NULL)
, _read_index(0)
, _read_offset(0)
, _read_size_max(RDEVICEGRID_READSIZE_MAX)
{
	for (int i = 0; i < 4; i++)
		_work_area[i].index = 0;
}

rDeviceUAVGrid::~rDeviceUAVGrid()
{
	delete[] _grid;
}

void rDeviceUAVGrid::onCreate(const rDeviceContext& rdc)
{
	RD_DEVICE_CLASS(Base)::onCreate(rdc);
}

void rDeviceUAVGrid::onInit()
{
	RD_DEVICE_CLASS(Base)::onInit();

	const TCHAR* prop;

	// CellSize
	prop = getProperty(_T("CELL_SIZE"));
	if (prop)
		_cell_size = (float)_tstof(prop);
	if (_cell_size < 0)
		_cell_size = 0.1f;

	// row/column size
	prop = getProperty(_T("row"));
	if (prop)
		_rows = _tstoi(prop);
	if (_rows < 0)
		_rows = 1;

	prop = getProperty(_T("col"));
	if (prop)
		_cols = _tstoi(prop);
	if (_cols < 0)
		_cols = 1;

	_cell_count = _cols * _rows;

	// Initialize grid
	_grid = new unsigned char[_cell_count];
	memset(_grid, 0x0, sizeof(unsigned char) * _cell_count);
	_cell_count_wa = _cell_count;
	_cell_count_occupied = 0;

	HeightMap hmap_wc;
	string_type hmap_path = _T("");
	float hmap_map_width = 0.0f;
	float hmap_map_length = 0.0f;
	prop = getProperty(_T("HMAP_PATH_WA")); if (prop) hmap_path = prop;
	prop = getProperty(_T("HMAP_W")); if (prop) hmap_map_width = (float)_tstof(prop);
	prop = getProperty(_T("HMAP_L")); if (prop) hmap_map_length = (float)_tstof(prop);
	if (!hmap_wc.Create(hmap_path, hmap_map_width, hmap_map_length, 0.0f, 1.0f))
	{
		_tprintf(_T("ERROR: rDeviceUAVGrid: failed to load work-area map (%s)\n"), hmap_path.c_str());
		//enable(false);
	}
	else
	{
		_cell_count_wa = 0;

		for (int row = 0; row < _rows; row++) {
			for (int col = 0; col < _cols; col++) {
				float x = (float)((col - (int)(_cols * 0.5f))) * _cell_size + (0.5f * _cell_size);
				float y = (float)((row - (int)(_rows * 0.5f))) * _cell_size + (0.5f * _cell_size);
				float height;
				hmap_wc.GetHeight(x, y, height);
				if (height >= 0.5f)
					_grid[row*_cols + col] |= 0x10; // mark it as a none-work-area-cell
				else
					_cell_count_wa++;
			}
		}
	}
	
	// tool info
	prop = getProperty(_T("TOOL_SYS_NAME"));
	if (prop)
		_tool_robotName = prop;

	prop = getProperty(_T("TOOL_BODY_NAME"));
	if (prop)
		_tool_bodyName = prop;

	prop = getProperty(_T("TOOL_DEVICE_NAME"));
	if (prop)
		_tool_deviceName = prop;

	prop = getProperty(_T("TOOL_DEVICE_NAME_PTO"));
	if (prop)
		_tool_deviceName_PTO = prop;

	_tool_bodyID = _rdc.m_deviceAPI->getBodyID(_tool_robotName.c_str(), _tool_bodyName.c_str());
	if (_tool_bodyID == INVALID_RID)
	{
		_tprintf(_T("ERROR: rDeviceUAVGrid: cannot find tool (system: %s, body: %s)\n"), _tool_robotName.c_str(), _tool_bodyName.c_str());
		enable(false);
	}

	_tool_width = 1.0f;

	_tool_deviceID = _rdc.m_deviceAPI->getDeviceID(_tool_robotName.c_str(), _tool_deviceName.c_str());
	if (_tool_deviceID == INVALID_RID)
	{
		_tprintf(_T("ERROR: rDeviceUAVGrid: cannot find tool. (system: %s, device: %s)\n"), _tool_robotName.c_str(), _tool_deviceName.c_str());
		enable(false);
	}
	else
	{
		TCHAR prop[1024];

		if (_rdc.m_deviceAPI->getDeviceProperty(_tool_deviceID, _T("TOOL_TYPE"), prop))
		{
			if (!_tcsicmp(prop, _T("line")))
				_tool_type = TOOL_LINE;
			else
				_tool_type = TOOL_NONE;
		}
		if (_tool_type == TOOL_NONE)
		{
			_tprintf(_T("ERROR: rDeviceUAVGrid: tool type is not known. (system: %s, device: %s)\n"), _tool_robotName.c_str(), _tool_deviceName.c_str());
			enable(false);
		}

		if (_rdc.m_deviceAPI->getDeviceProperty(_tool_deviceID, _T("TOOL_OFFSET_R"), prop))
		{
			float R[9];
			const TCHAR* prop_ptr = (const TCHAR*)prop;
			parse_vector(R, 9, prop_ptr);
			_tool_offset.linear() << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
		}
		if (_rdc.m_deviceAPI->getDeviceProperty(_tool_deviceID, _T("TOOL_OFFSET_r"), prop))
		{
			float r[3];
			const TCHAR* prop_ptr = (const TCHAR*)prop;
			parse_vector(r, 3, prop_ptr);
			_tool_offset.translation() << r[0], r[1], r[2];
		}

		// width of worker(tool)
		if (_rdc.m_deviceAPI->getDeviceProperty(_tool_deviceID, _T("TOOL_WIDTH"), prop))
			_tool_width = (float)_tstof(prop);
		if (_tool_width < 0)
			_tool_width = 1.0f;
	}

	_tool_deviceID_PTO = _rdc.m_deviceAPI->getDeviceID(_tool_robotName.c_str(), _tool_deviceName_PTO.c_str());
	if (_tool_deviceID_PTO == INVALID_RID)
	{
		_tprintf(_T("ERROR: rDeviceUAVGrid: cannot find tool PTO. (system: %s, device: %s)\n"), _tool_robotName.c_str(), _tool_deviceName_PTO.c_str());
		enable(false);
	}

	// maximum buffer size to read data once
	prop = getProperty(_T("READ_SIZE_MAX"));
	if (prop)
		_read_size_max = min(_tstoi(prop), RDEVICEGRID_READSIZE_MAX);
}

void rDeviceUAVGrid::onTerminate()
{
	RD_DEVICE_CLASS(Base)::onTerminate();
}

int rDeviceUAVGrid::readDeviceValue(void* buffer, int len, int port)
{
	int read = 0;

	if (len > (int)(_read_size_max + sizeof(int) * 3))
	{
		_lock.lock();
		{
			int datasize = (int)(sizeof(unsigned char) * (_rows * _cols)) - _read_offset;
			if (datasize < 0)
			{
				read = 0;
			}
			else if (datasize <= _read_size_max)
			{
				((int*)buffer)[0] = _read_index;
				((int*)buffer)[1] = _read_offset;
				((int*)buffer)[2] = datasize;
				memcpy((char*)buffer + 12, _grid + _read_offset, datasize);
				_read_index = 0;
				_read_offset = 0;
				read = datasize + 12;
			}
			else
			{
				datasize = _read_size_max;
				((int*)buffer)[0] = _read_index;
				((int*)buffer)[1] = _read_offset;
				((int*)buffer)[2] = datasize;
				memcpy((char*)buffer + 12, _grid + _read_offset, datasize);
				_read_index += 1;
				_read_offset += datasize;
				read = datasize + 12;
			}
		}
		_lock.unlock();
		return read;
	}
	else
		return 0;
}

int rDeviceUAVGrid::writeDeviceValue(void* buffer, int len, int port)
{
	return 0;
}

int rDeviceUAVGrid::command(int cmd, int arg, void* udata, int port)
{
	switch (cmd)
	{
	case UAVDRV_DATAPORT_RESET_WORK:
	{
		_lock.lock();
		{
			_resetWork();
		}
		_lock.unlock();
	}
	break;

	case UAVDRV_DATAPORT_SET_WORKAREA:
	{
		if (!udata)
			break;

		WORKAREAPOINT* pt = (WORKAREAPOINT*)udata;
		if (pt->index < 1 || pt->index > 4)
			break;

		_lock.lock();
		{
			memcpy(&_work_area[pt->index - 1], pt, sizeof(WORKAREAPOINT));
			_updateWorkArea();
		}
		_lock.unlock();
	}
	break;
	}

	return 0;
}

int rDeviceUAVGrid::monitorDeviceValue(void* buffer, int len, int port)
{
	if (len == (int)(sizeof(int) * 2))
	{ // read (occupied cell count , whole cell count)
		_lock.lock();
		((int*)buffer)[0] = _cell_count_wa;
		((int*)buffer)[1] = _cell_count_occupied;
		_lock.unlock();
		return len;
	}
	else
		return 0;
}

void rDeviceUAVGrid::importDevice(rTime time, void* mem)
{
	float r_w[3];
	float R_w[9];

	_lock.lock();
	{
		_rdc.m_deviceAPI->readDeviceValue(_tool_deviceID_PTO, &_tool_activated, 1);

		if (_tool_activated)
		{
			_rdc.m_deviceAPI->getBodyPosition(_tool_bodyID, r_w);
			_rdc.m_deviceAPI->getBodyOrientation(_tool_bodyID, R_w);
			_tool_bodyHT.linear() << R_w[0], R_w[1], R_w[2], 
									 R_w[3], R_w[4], R_w[5], 
									 R_w[6], R_w[7], R_w[8];
			_tool_bodyHT.translation() << r_w[0], r_w[1], r_w[2];

			HTransform Tg = _tool_bodyHT * _tool_offset;
			Displacement v1(-_tool_width*0.5, 0, 0);
			Displacement v2( _tool_width*0.5, 0, 0);
			Displacement v1g = Tg * v1;//Tg.Transform(v1);
			Displacement v2g = Tg * v2;//Tg.Transform(v2);
			Vector3D dir = v2g - v1g;
			Vector3D p;
			dir.normalize();
			// to calculate number of segments,
			// we multiply by 2 considering the tool is lying in diagonal direction of the cell.
			unsigned int nSegments = static_cast<unsigned int>(2.0*_tool_width/_cell_size);
			float segLength = (nSegments > 0 ? _tool_width / (float)nSegments : _tool_width);
			for (unsigned int i = 0; i <= nSegments; ++i)
			{
				p = v1g + dir*i*segLength;

				int col = (int)(p[0] / _cell_size) + (int)(_cols * 0.5) + (p[0] < 0 ? -1 : 0);
				int row = (int)(p[1] / _cell_size) + (int)(_rows * 0.5) + (p[1] < 0 ? -1 : 0); 

				if (col >= 0 && col < _cols && 
					row >= 0 && row < _rows &&
					0x00 == _grid[row*_cols + col]) 
				{
					_grid[row*_cols + col] = 0x1;
					_cell_count_occupied++;
				}
			}
		}
	}
	_lock.unlock();
}

void rDeviceUAVGrid::_resetWork()
{
	//_read_offset = 0;
	//_read_index = 0;
	for (int i = 0; i < _cell_count; i++)
		_grid[i] &= 0xf0;
	_cell_count_occupied = 0;
}

void rDeviceUAVGrid::_updateWorkArea()
{
	// check if we have all work-area corners
	for (int i = 0; i < 4; i++) {
		if (_work_area[i].index != (i + 1))
			return;
	}

	_cell_count_wa = 0;
	_cell_count_occupied = 0;
	
	auto f = [&](float x1, float y1, float x2, float y2, float x, float y) -> bool {
		float th = 1.0e-5;
		return ((abs(x1 - x2) < th)
			? (x < x1)
			: (((y2 - y1) / (x2 - x1) * (x - x1) + y1 - y) > 0.0f));
	};

	for (int row = 0; row < _rows; row++) {
		for (int col = 0; col < _cols; col++) {
			float x = (float)((col - (int)(_cols * 0.5f))) * _cell_size + (0.5f * _cell_size);
			float y = (float)((row - (int)(_rows * 0.5f))) * _cell_size + (0.5f * _cell_size);

			bool w1 = f(_work_area[0].x, _work_area[0].y, _work_area[1].x, _work_area[1].y, x, y);
			bool w2 = f(_work_area[2].x, _work_area[2].y, _work_area[3].x, _work_area[3].y, x, y);
			bool w3 = f(_work_area[0].x, _work_area[0].y, _work_area[3].x, _work_area[3].y, x, y);
			bool w4 = f(_work_area[1].x, _work_area[1].y, _work_area[2].x, _work_area[2].y, x, y);
			bool wa = (w1 != w2) && (w3 != w4);

			if (wa) {
				_grid[row*_cols + col] &= 0x0f;
				_cell_count_wa++;
				if (_grid[row*_cols + col] & 0x0f)
					_cell_count_occupied++;
			}
			else {
				_grid[row*_cols + col] = 0x10;
			}
		}
	}
}

} // namespace plugin
} // namespace rlab
