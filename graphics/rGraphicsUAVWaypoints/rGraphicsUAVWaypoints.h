/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#ifndef __RGRAPHICSUAVWAYPOINTS_H__
#define __RGRAPHICSUAVWAYPOINTS_H__

#include <rlab/plugin/rGraphics.h>
#include <rlab/plugin/rGraphicsAPI.h>
#include <list>
#include "HeightMap.h"

namespace rlab {
namespace plugin {

class rGraphicsUAVWaypoints : public rGraphics
{
public:
	/** Constructor	*/
	rGraphicsUAVWaypoints();

	/** Destructor	*/
	virtual ~rGraphicsUAVWaypoints();

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
	float _wp_color[4];
	float _wp_size;
	string_type _wp_label_format;
	float _wp_label_color[4];
	float _wp_label_pos[3];
	float _wp_label_size;
	float _line_color[4];
	float _line_width;
	float _offset_z;
	bool _wp_visible;
	
	int _hPath;
	std::list<int> _hWP;
	int* _indexArray;
	float* _vertexArray;
	int _wp_count;

	HeightMap _hmap;
};

} // namespace plugin
} // namespace rlab

#endif // __RGRAPHICSUAVWAYPOINTS_H__
