/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */

#ifndef __rlab_HEIGHTMAP_H__
#define __rlab_HEIGHTMAP_H__

#include <rlab/type.h>

enum ePredefinedHeightMapType
{
	HM_FLAT,
	HM_SLOPE_X_INC,
	HM_SLOPE_Y_INC,
	HM_SLOPE_XY_INC
};

#define PREDEFINE_HBITMAP_WIDTH  101
#define PREDEFINE_HBITMAP_LENGTH 101

class HeightMap
{
public:
	HeightMap();
	~HeightMap();

	bool Create(const string_type& path, float width, float length, float height_min, float height_max);
	bool Create(const ePredefinedHeightMapType hm_type, float width, float length, float height_min, float height_max);
	bool GetHeight(float x, float y, float& height);
	bool GetNormal(float x, float y, float normal[3]);
	bool GetNormalEx(float x, float y, int wnd_size, float normal[3]);

private:
	void _CalculateVertexNormal();

private:
	string_type				_path;
	float					_map_width;
	float					_map_length;
	float					_map_height_min;
	float					_map_height_max;
	float					_map_width_scale_mpp;	// scale in meter per pixel
	float					_map_length_scale_mpp;
	float					_map_height_scale_mpp;
	float*					_heights;				// vertex height(z)
	float*					_normals;				// vertex normal
	int						_bmp_pixel_width;
	int						_bmp_pixel_length;
};

#endif
