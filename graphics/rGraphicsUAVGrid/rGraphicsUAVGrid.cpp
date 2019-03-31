/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#include <rlab/type.h>
#include <rlab/const.h>
#include <rlab/utils/rParseUtil.h>
#include <rlab/math/rMath.h>
#include "rGraphicsUAVGrid.h"

namespace rlab {
namespace plugin {

rGraphicsUAVGrid::rGraphicsUAVGrid()
: _hGrid(0)
, _hGridOccupied(0)
, _hNormal(0)
, _byTrace(NULL)
, _cols(0)
, _rows(0)
, _cell_count_total(0)
, _cell_count_occupied(0)
, _cell_coord(NULL)
, _cell_size(1)
, _grid_coord_z(0.0f)
, _grid_coord_z_occupied(0.0f)
, _grid_visible(false)
, _grid_visible_occupied(false)
, _grid_visible_normal(false)
, _indexArray(NULL)
, _indexArrayOccupied(NULL)
, _indexArrayNormal(NULL)
, _vertexArray(NULL)
, _vertexArrayOccupied(NULL)
, _vertexArrayNormal(NULL)
{
	_grid_color[0] = _grid_color[1] = _grid_color[2] = _grid_color[3] = 1.0f;
	_grid_color_occupied[0] = _grid_color_occupied[1] = _grid_color_occupied[2] = _grid_color_occupied[3] = 1.0f;
}

rGraphicsUAVGrid::~rGraphicsUAVGrid()
{
	if (_byTrace) delete[] _byTrace;
	if (_cell_coord) delete[] _cell_coord;
	if (_indexArray) delete[] _indexArray;
	if (_indexArrayOccupied) delete[] _indexArrayOccupied;
	if (_vertexArray) delete[] _vertexArray;
	if (_vertexArrayOccupied) delete[] _vertexArrayOccupied;
	if (_indexArrayNormal) delete[] _indexArrayNormal;
	if (_vertexArrayNormal) delete[] _vertexArrayNormal;
}

void rGraphicsUAVGrid::onCreate(double dTime)
{
	const wchar_t* prop;
	int cell_index;
	string_type hmap_path;
	float hmap_map_width;
	float hmap_map_length;
	float hmap_map_height_min;
	float hmap_map_height_max;
	
	prop = getProperty(_T("CELL_SIZE")); _cell_size = (float)_tstof(prop);
	prop = getProperty(_T("GRID_HEIGHT")); _grid_coord_z = (float)_tstof(prop);
	prop = getProperty(_T("GRID_HEIGHT_OCCUPIED")); _grid_coord_z_occupied = (float)_tstof(prop);
	prop = getProperty(_T("GRID_COLOR")); parse_vector(_grid_color, 4, prop);
	prop = getProperty(_T("GRID_COLOR_OCCUPIED")); parse_vector(_grid_color_occupied, 4, prop);
	prop = getProperty(_T("GRID_VISIBLE")); if (!_tcsicmp(prop, _T("true")) || !_tcsicmp(prop, _T("yes"))) _grid_visible = true;
	prop = getProperty(_T("GRID_VISIBLE_OCCUPIED")); if (!_tcsicmp(prop, _T("true")) || !_tcsicmp(prop, _T("yes"))) _grid_visible_occupied = true;
	prop = getProperty(_T("GRID_VISIBLE_NORMAL")); if (!_tcsicmp(prop, _T("true")) || !_tcsicmp(prop, _T("yes"))) _grid_visible_normal = true;
	prop = getProperty(_T("col")); _cols = _tstoi(prop);
	prop = getProperty(_T("row")); _rows = _tstoi(prop);
	prop = getProperty(_T("HMAP_PATH")); hmap_path = prop;
	prop = getProperty(_T("HMAP_W")); hmap_map_width = (float)_tstof(prop);
	prop = getProperty(_T("HMAP_L")); hmap_map_length = (float)_tstof(prop);
	prop = getProperty(_T("HMAP_H_min")); hmap_map_height_min = (float)_tstof(prop);
	prop = getProperty(_T("HMAP_H_max")); hmap_map_height_max = (float)_tstof(prop);

	// initiate height map
	if (!_hmap.Create(hmap_path, hmap_map_width, hmap_map_length, hmap_map_height_min, hmap_map_height_max))
	{
		assert(0 && "ERROR! rGraphicsUAVGrid: failed to load height map.\n");
	}

	if (_cols > 0 && _rows > 0)
	{
		_cell_count_total = _rows * _cols;
		_cell_count_occupied = 0;
		_byTrace = new unsigned char[_cell_count_total];
		memset(_byTrace, 0, sizeof(_byTrace[0])*_cell_count_total);

		_cell_coord = new Coord3D[_cell_count_total];
		for (int row=0; row<_rows; row++) {
			for (int col=0; col<_cols; col++) {
				cell_index = row * _cols + col;
				_cell_coord[cell_index][0] = (float)((col - (int)(_cols * 0.5)) + 0.5) * _cell_size;
				_cell_coord[cell_index][1] = (float)((row - (int)(_rows * 0.5)) + 0.5) * _cell_size;
				_cell_coord[cell_index][2] = 0.0f;
			}
		}

		int vertexCnt = (_rows +1) * (_cols + 1);
		int quadCnt = _rows * _cols;
		int vertex_index = 0;
		
		// index/vertex array to draw grid.
		_indexArray = new int[quadCnt*8];			// 8-vertex per a quad (4 lines)
		_vertexArray = new float[vertexCnt*3];		// 3D coord per a vertex

		// index/vertex array to draw vertex normal.
		_indexArrayNormal = new int[vertexCnt*2];		// 2-vertex(normal vector) per a vertex
		_vertexArrayNormal = new float[vertexCnt*2*3];	// 3D coord per a vertex
		
		// index/vertex array to draw work-cells.
		_indexArrayOccupied = new int[quadCnt*8];	// 8-vertex per a quad (4 lines)
		_vertexArrayOccupied = new float[vertexCnt*3];

		// color array
		float vertexColor[4] = {0, 0, 0, 1.0f};
		float vertex_coord_x, vertex_coord_y;
		float height;
		float normal[3], normal_len = 0.2f;

		for (int row=0; row<_rows+1; row++) {
			for (int col=0; col<_cols+1; col++) {
				vertex_index = row * (_cols + 1) + col;
				vertex_coord_x = (float)((col - (int)(_cols * 0.5))) * _cell_size;
				vertex_coord_y = (float)((row - (int)(_rows * 0.5))) * _cell_size;
				_hmap.GetHeight(vertex_coord_x, vertex_coord_y, height);

				_vertexArray[3*vertex_index+0] = vertex_coord_x;
				_vertexArray[3*vertex_index+1] = vertex_coord_y;
				_vertexArray[3*vertex_index+2] = _grid_coord_z + height;
				_vertexArrayOccupied[3*vertex_index+0] = vertex_coord_x;
				_vertexArrayOccupied[3*vertex_index+1] = vertex_coord_y;
				_vertexArrayOccupied[3*vertex_index+2] = _grid_coord_z_occupied + height;

				_hmap.GetNormalEx(vertex_coord_x, vertex_coord_y, 1, normal);
				_vertexArrayNormal[6*vertex_index+0] = vertex_coord_x;
				_vertexArrayNormal[6*vertex_index+1] = vertex_coord_y;
				_vertexArrayNormal[6*vertex_index+2] = _grid_coord_z + height;
				_vertexArrayNormal[6*vertex_index+3] = vertex_coord_x + normal[0]*normal_len;
				_vertexArrayNormal[6*vertex_index+4] = vertex_coord_y + normal[1]*normal_len;
				_vertexArrayNormal[6*vertex_index+5] = _grid_coord_z + height + normal[2]*normal_len;

				_indexArrayNormal[2*vertex_index+0] = 2*vertex_index+0;
				_indexArrayNormal[2*vertex_index+1] = 2*vertex_index+1;
			}
		}

		vertex_index = 0;
		for (int row=0; row<_rows; row++) {
			for (int col=0; col<_cols; col++) {
				_indexArray[vertex_index++] = row * (_cols + 1) + col;
				_indexArray[vertex_index++] = row * (_cols + 1) + col + 1;
				_indexArray[vertex_index++] = row * (_cols + 1) + col + 1;
				_indexArray[vertex_index++] = (row + 1) * (_cols + 1) + col + 1;
				_indexArray[vertex_index++] = (row + 1) * (_cols + 1) + col + 1;
				_indexArray[vertex_index++] = (row + 1) * (_cols + 1) + col;
				_indexArray[vertex_index++] = (row + 1) * (_cols + 1) + col;
				_indexArray[vertex_index++] = row * (_cols + 1) + col;
			}
		}

		if (_grid_visible) {
			_hGrid = makeMesh();
			setVertexArray(_hGrid, _vertexArray, vertexCnt);
			setVertexIndices(_hGrid, _indexArray, quadCnt*8);
			setColorArray(_hGrid, vertexColor, 1);
			setColorBinding(_hGrid, rGraphicsAPI::BIND_OVERALL);
			enableLighting(_hGrid, false);
			setMode(_hGrid, rGraphicsAPI::LINES/*QUADS*/);
			show(_hGrid, true);
		}

		if (_grid_visible_occupied) {
			_hGridOccupied = makeMesh();
			setVertexArray(_hGridOccupied, _vertexArray, 0);
			setVertexIndices(_hGridOccupied, _indexArrayOccupied, 0);
			setColorArray(_hGridOccupied, vertexColor, 1);
			setColorBinding(_hGridOccupied, rGraphicsAPI::BIND_OVERALL);
			enableLighting(_hGridOccupied, false);
			setMode(_hGridOccupied, rGraphicsAPI::LINES/*QUADS*/);
			show(_hGridOccupied, true);
		}

		if (_grid_visible_normal) {
			_hNormal = makeMesh();
			setVertexArray(_hNormal, _vertexArrayNormal, vertexCnt*2);
			setVertexIndices(_hNormal, _indexArrayNormal, vertexCnt*2);
			setColorArray(_hNormal, vertexColor, 1);
			setColorBinding(_hNormal, rGraphicsAPI::BIND_OVERALL);
			enableLighting(_hNormal, false);
			setMode(_hNormal, rGraphicsAPI::LINES/*QUADS*/);
		}
	}
}

void rGraphicsUAVGrid::onDestroy(double dTime)
{
}

void rGraphicsUAVGrid::onRender(double dTime)
{
	if (_cell_count_total <= 0) return;

	/*float R[9], r[3];
	float cellRot[9], cellPos[3];
	HTransform bodyT; bodyT.setIdentity();
	HTransform bodyT_inv; bodyT_inv.setIdentity();
	Displacement cellPos_g; cellPos_g.setZero();
	Displacement cellPos_b; cellPos_b.setZero();

	getHTransform(r, R, true);
	bodyT.linear() << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
	bodyT.translation() << r[0], r[1], r[2];
	bodyT_inv = bodyT.inverse();

	cellRot[0] = (float)bodyT_inv.linear()(0,0);
	cellRot[1] = (float)bodyT_inv.linear()(0,1);
	cellRot[2] = (float)bodyT_inv.linear()(0,2);
	cellRot[3] = (float)bodyT_inv.linear()(1,0);
	cellRot[4] = (float)bodyT_inv.linear()(1,1);
	cellRot[5] = (float)bodyT_inv.linear()(1,2);
	cellRot[6] = (float)bodyT_inv.linear()(2,0);
	cellRot[7] = (float)bodyT_inv.linear()(2,1);
	cellRot[8] = (float)bodyT_inv.linear()(2,2);

	cellPos[0] = (float)bodyT_inv.translation()[0];
	cellPos[1] = (float)bodyT_inv.translation()[1];
	cellPos[2] = (float)bodyT_inv.translation()[2];*/

	if (_hGrid) {
		setColor(_hGrid, _grid_color[0], _grid_color[1], _grid_color[2], _grid_color[3]);
		//translate(_hGrid, (float*)cellPos);
		//rotate(_hGrid, (float*)cellRot);
		if (_grid_visible)
			show(_hGrid, true); // force to show grid even though user turns off visiblity in rPlayer.
	}

	if (_hNormal) {
		if (!_grid_visible_normal) {
			show(_hNormal, false);
		}
		else {
			setColor(_hNormal, _grid_color[0], _grid_color[1], _grid_color[2], _grid_color[3]);
			//translate(_hNormal, (float*)cellPos);
			//rotate(_hNormal, (float*)cellRot);
		}
	}

	if (_hGridOccupied) {
		int vertexCnt = (_rows +1) * (_cols + 1);
		int quadCnt = _rows * _cols;
		int vertex_index = 0;
		int quad_index = 0;
		float vertexColor[4] = {0, 0, 0, 1.0f};

		for (int row=0; row<_rows; row++) {
			for (int col=0; col<_cols; col++) {

				quad_index = row * _cols + col;
			
				if (!_byTrace[quad_index])
					continue;

				_indexArrayOccupied[vertex_index++] = row * (_cols + 1) + col;
				_indexArrayOccupied[vertex_index++] = row * (_cols + 1) + col + 1;
				_indexArrayOccupied[vertex_index++] = row * (_cols + 1) + col + 1;
				_indexArrayOccupied[vertex_index++] = (row + 1) * (_cols + 1) + col + 1;
				_indexArrayOccupied[vertex_index++] = (row + 1) * (_cols + 1) + col + 1;
				_indexArrayOccupied[vertex_index++] = (row + 1) * (_cols + 1) + col;
				_indexArrayOccupied[vertex_index++] = (row + 1) * (_cols + 1) + col;
				_indexArrayOccupied[vertex_index++] = row * (_cols + 1) + col;
			}
		}

		if (vertex_index > 0)
		{
			setVertexArray(_hGridOccupied, _vertexArrayOccupied, vertexCnt);
			setVertexIndices(_hGridOccupied, _indexArrayOccupied, vertex_index);
			setColorArray(_hGridOccupied, vertexColor, 1);
			setColorBinding(_hGridOccupied, rGraphicsAPI::BIND_OVERALL);
			enableLighting(_hGridOccupied, false);
			setMode(_hGridOccupied, rGraphicsAPI::LINES/*QUADS*/);

			setColor(_hGridOccupied, _grid_color_occupied[0], _grid_color_occupied[1], _grid_color_occupied[2], _grid_color_occupied[3]);
			//translate(_hGridOccupied, (float*)cellPos);
			//rotate(_hGridOccupied, (float*)cellRot);
		
			if (_grid_visible_occupied)
				show(_hGridOccupied, true); // force to show grid even though user turns off visiblity in rPlayer.
		}
		else
		{
			show(_hGridOccupied, false);
		}
	}
}

void rGraphicsUAVGrid::onDevice(double dTime, DWORD dSize, BYTE* dData)
{
	if (_cell_count_total <= 0 || dSize < 12) return;

	int read_index = ((int*)dData)[0];
	int read_offset = ((int*)dData)[1];
	int read_size = ((int*)dData)[2];

	if (read_offset < 0 || 
		read_offset >= _cell_count_total ||
		read_offset + read_size > _cell_count_total)
		return;

	for (int i=0; i<read_size; i++)
	{
		if (!_byTrace[read_offset+i] && (dData+12)[i])
		{
			_cell_count_occupied++;
			_byTrace[read_offset+i] = (dData+12)[i];
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
	return new rGraphicsUAVGrid();								
}

void get_rGraphicsFactory(rGraphicsFactory** factory)			
{															
	*factory = (rGraphicsFactory*)new GraphicsFactory();		
}

} // namespace plugin
} // namespace rlab
