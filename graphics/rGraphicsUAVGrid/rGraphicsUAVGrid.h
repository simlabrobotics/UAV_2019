/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#ifndef __RGRAPHICSUAVGRID_H__
#define __RGRAPHICSUAVGRID_H__

#include <rlab/plugin/rGraphics.h>
#include <rlab/plugin/rGraphicsAPI.h>
#include "HeightMap.h"

namespace rlab {
namespace plugin {

class rGraphicsUAVGrid : public rGraphics
{
public:
	/** Constructor	*/
	rGraphicsUAVGrid();

	/** Destructor	*/
	virtual ~rGraphicsUAVGrid();

	/** Callback function when Instance of rCustomDraw is created*/
	virtual void onCreate(double dTime);

	/** Callback function when Instance of rCustomDraw is destroyed*/
	virtual void onDestroy(double dTime);

	/** Callback function when Device update*/
	virtual void onDevice(double dTime, DWORD dSize, BYTE* dData);

	/** Callback function when Touch update	*/
	virtual void onTouch(double dTime, int bodyID, int command, bool bOn) {};

	/** Callback function for rendering */
	virtual void onRender(double dTime);

	/** Callback function when timer event happened */
	virtual void onTimer(double dTime, int wParam, int lParam) {};

	/** Callback function, It is called when user event is happened*/
	virtual void onEvent(double dTime, int wParam, int lParam) {};

	/** Callback function, It is called when preiodically by rPlayer for treating GUI.*/
	virtual void onGUI(double dTime, bool bShow = true) {};

private:
	unsigned char* _byTrace;
	int _cols;
	int _rows;
	int _cell_count;
	float _cell_size;
	float _grid_coord_z;
	float _grid_color[4];
	float _grid_color_wa[4];
	float _grid_coord_z_occupied;
	float _grid_color_occupied[4];
	bool _grid_visible;
	bool _grid_visible_occupied;
	bool _grid_visible_normal;

	int _hGrid;
	int* _indexArray;
	float* _vertexArray;
	float* _normalArray;

	int _hGridWorkArea;
	int* _indexArrayWorkArea;
	float* _vertexArrayWorkArea;

	int _hGridOccupied;
	int* _indexArrayOccupied;
	float* _vertexArrayOccupied;

	/*struct Coord3D
	{
		float _xyz[3];
		float& operator[] (const size_t index) { return _xyz[index]; }
	};
	Coord3D* _cell_coord;*/

	HeightMap _hmap;

	int _hNormal;
	int* _indexArrayNormal;
	float* _vertexArrayNormal;
};

} // namespace plugin
} // namespace rlab

#endif // __RGRAPHICSUAVGRID_H__
