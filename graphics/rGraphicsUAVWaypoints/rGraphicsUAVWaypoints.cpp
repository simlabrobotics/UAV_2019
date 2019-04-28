/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#include <rlab/type.h>
#include <rlab/const.h>
#include <rlab/utils/rParseUtil.h>
#include <rlab/math/rMath.h>
#include "rGraphicsUAVWaypoints.h"
#include "UAV_def.h"

namespace rlab {
namespace plugin {

rGraphicsUAVWaypoints::rGraphicsUAVWaypoints()
: _hPath(-1)
, _indexArray(NULL)
, _vertexArray(NULL)
{
	_wp_label_format = _T("wp_%d");
	_wp_label_pos[0] = _wp_label_pos[1] = _wp_label_pos[2] = 0.0f;
	_wp_label_color[0] = _wp_label_color[1] = _wp_label_color[2] = _wp_label_color[3] = 1.0f;
	_wp_label_size = 20;
	_wp_color[0] = _wp_color[1] = _wp_color[2] = _wp_color[3] = 1.0f;
	_wp_size = 0.5f;
	_line_color[0] = _line_color[1] = _line_color[2] = _line_color[3] = 1.0f;
	_line_width = 1.0f;
	_offset_z = 0.0f;
	_wp_visible = true;
	_wp_count = 0;
}

rGraphicsUAVWaypoints::~rGraphicsUAVWaypoints()
{
	if (_indexArray) delete[] _indexArray;
	if (_vertexArray) delete[] _vertexArray;

	// remote all waypoint markers:
	for (std::list<int>::iterator itr_wp = _hWP.begin();
			itr_wp != _hWP.end();
			itr_wp++) {
				removeLabel(*itr_wp);
				remove(*itr_wp);
	}
	_hWP.clear();
}

void rGraphicsUAVWaypoints::onCreate(double dTime)
{
	const wchar_t* prop;
	string_type hmap_path;
	float hmap_map_width;
	float hmap_map_length;
	float hmap_map_height_min;
	float hmap_map_height_max;
	
	prop = getProperty(_T("WP_LABEL_FORMAT")); _wp_label_format = prop;
	prop = getProperty(_T("WP_LABEL_COLOR")); parse_vector(_wp_label_color, 4, prop);
	prop = getProperty(_T("WP_COLOR")); parse_vector(_wp_color, 4, prop);
	prop = getProperty(_T("WP_SIZE")); _wp_size = (float)_tstof(prop);
	prop = getProperty(_T("LINE_COLOR")); parse_vector(_line_color, 4, prop);
	prop = getProperty(_T("LINE_WIDTH")); _line_width = (float)_tstof(prop);
	prop = getProperty(_T("OFFSET_FROM_GROUND")); _offset_z = (float)_tstof(prop);
	
	prop = getProperty(_T("HMAP_PATH")); hmap_path = prop;
	prop = getProperty(_T("HMAP_W")); hmap_map_width = (float)_tstof(prop);
	prop = getProperty(_T("HMAP_L")); hmap_map_length = (float)_tstof(prop);
	prop = getProperty(_T("HMAP_H_min")); hmap_map_height_min = (float)_tstof(prop);
	prop = getProperty(_T("HMAP_H_max")); hmap_map_height_max = (float)_tstof(prop);

	// initiate height map
	if (!_hmap.Create(hmap_path, hmap_map_width, hmap_map_length, hmap_map_height_min, hmap_map_height_max))
	{
		assert(0 && "ERROR! rGraphicsUAVWaypoints: failed to load height map.\n");
	}




	_wp_count = 3;
		
	// index/vertex array to draw waypoints.
	_indexArray = new int[_wp_count];
	_vertexArray = new float[_wp_count*3];


	_vertexArray[0] = 0.0;
	_vertexArray[1] = 3.0;
	_vertexArray[2] = 0.0;
	_vertexArray[3] = 0.0;
	_vertexArray[4] = 6.0;
	_vertexArray[5] = 0.0;
	_vertexArray[6] = 3.0;
	_vertexArray[7] = 9.0;
	_vertexArray[8] = 0.0;
		
	// color array
	float vertexColor[4] = {1, 1, 0, 1.0f};
	float height;

	for (int vertex_index=0; vertex_index<_wp_count; vertex_index++) {
		_hmap.GetHeight(_vertexArray[3*vertex_index+0], _vertexArray[3*vertex_index+1], height);
		_vertexArray[3*vertex_index+2] = height + _offset_z;
		_indexArray[vertex_index] = vertex_index;
	}

	if (_wp_visible) {
		_hPath = makeMesh();
		setVertexArray(_hPath, _vertexArray, _wp_count);
		setVertexIndices(_hPath, _indexArray, _wp_count);
		setColorArray(_hPath, vertexColor, 1);
		setColorBinding(_hPath, rGraphicsAPI::BIND_OVERALL);
		enableLighting(_hPath, false);
		setMode(_hPath, rGraphicsAPI::LINE_STRIP);
		setColor(_hPath, _line_color[0], _line_color[1], _line_color[2], _line_color[3]);
		setLineWidth(_hPath, _line_width);
		show(_hPath, true);
		//makeLabel(_hPath, _T(""));
	}

	// create sample waypoint marker:
	for (int vertex_index=0; vertex_index<_wp_count; vertex_index++) {
		int marker = cube(_wp_size);
		translate(marker, &_vertexArray[3*vertex_index+0]);
		setColor(marker, _wp_color);
		TCHAR wp_name[16] = {0, };
		_stprintf_s(wp_name, _wp_label_format.c_str(), vertex_index+1);
		makeLabel(marker, wp_name);
		setLabelColor(marker, _wp_label_color);
		setLabelPos(marker, _wp_label_pos);
		setLabelSize(marker, _wp_label_size);
		_hWP.push_back(marker);
	}
}

void rGraphicsUAVWaypoints::onDestroy(double dTime)
{
}

void rGraphicsUAVWaypoints::onRender(double dTime)
{
	if (!_wp_visible)
		return;

	if (_hPath >= 0) {
		setVertexArray(_hPath, _vertexArray, _wp_count);
		setVertexIndices(_hPath, _indexArray, _wp_count);
		setColorArray(_hPath, _wp_color, 1);
		setColorBinding(_hPath, rGraphicsAPI::BIND_OVERALL);
		enableLighting(_hPath, false);
		setMode(_hPath, rGraphicsAPI::LINE_STRIP);
		setColor(_hPath, _line_color[0], _line_color[1], _line_color[2], _line_color[3]);
		setLineWidth(_hPath, _line_width);
		show(_hPath, true); // force to show grid even though user turns off visiblity in rPlayer.
	}

	for (std::list<int>::iterator itr_wp = _hWP.begin();
		 itr_wp != _hWP.end();
		 itr_wp++) {
			 setColor(*itr_wp, _wp_color);
			 show(*itr_wp);
	}

	/// XXX: TEST graphics:
	/*if (_hCube < 0) {
		_hCube = cube(1.0f);
		float cPos[3] = {0, 0, 3};
		translate(_hCube, cPos);
		makeLabel(_hCube, L"CUBE");
		float cLabelColor[4] = {1, 1, 0, 1.0f};
		setLabelColor(_hCube, cLabelColor);
		float cLabelPos[3] = {0, 0, 2};
		setLabelPos(_hCube, cLabelPos);
		setLabelSize(_hCube, 50);
	}*/
}

void rGraphicsUAVWaypoints::onDevice(double dTime, DWORD dSize, BYTE* dData)
{
	if (dSize < 4) return;

	int wp_count = ((int*)dData)[0];
	if (wp_count > 0)
	{
		// remote vertex and index buffers:
		if (_indexArray) delete[] _indexArray;
		if (_vertexArray) delete[] _vertexArray;

		// remote all waypoint markers:
		for (std::list<int>::iterator itr_wp = _hWP.begin();
			 itr_wp != _hWP.end();
			 itr_wp++) {
				 removeLabel(*itr_wp);
				 remove(*itr_wp);
		}
		_hWP.clear();

		_wp_count = wp_count;
		_indexArray = new int[wp_count];
		_vertexArray = new float[wp_count*3];
		float height;
		WAYPOINT* pwp = (WAYPOINT*)((int*)dData + 1);

		for (int vertex_index=0; vertex_index<wp_count; vertex_index++) {
			// set vertex array:
			_vertexArray[3*vertex_index+0] = pwp->x;
			_vertexArray[3*vertex_index+1] = pwp->y;
			_hmap.GetHeight(_vertexArray[3*vertex_index+0], _vertexArray[3*vertex_index+1], height);
			_vertexArray[3*vertex_index+2] = height + _offset_z;
			
			// set index array:
			_indexArray[vertex_index] = vertex_index;
			
			// create waypoint marker:
			int marker = cube(_wp_size);
			translate(marker, &_vertexArray[3*vertex_index+0]);
			setColor(marker, _wp_color);
			TCHAR wp_name[16] = {0, };
			_stprintf_s(wp_name, _wp_label_format.c_str(), pwp->index);
			makeLabel(marker, wp_name);
			setLabelColor(marker, _wp_label_color);
			setLabelPos(marker, _wp_label_pos);
			setLabelSize(marker, _wp_label_size);
			_hWP.push_back(marker);

			// move on to the next waypoint:
			pwp++;
		}
	}
}

class GraphicsFactory : public rGraphicsFactory					
{															
	void release();											
	rGraphics* createGraphics();								
};		

void GraphicsFactory::release()								
{															
	delete this;											
}

rGraphics* GraphicsFactory::createGraphics()						
{															
	return new rGraphicsUAVWaypoints();								
}

void get_rGraphicsFactory(rGraphicsFactory** factory)			
{															
	*factory = (rGraphicsFactory*)new GraphicsFactory();		
}

} // namespace plugin
} // namespace rlab
