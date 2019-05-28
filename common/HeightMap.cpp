/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */

#include "HeightMap.h"
#include <rlab/math/rMath.h>
#include <windows.h>
#include <assert.h>
#include <math.h>

#define ALIGNEDWIDTHBYTES(bits) (((bits)+31)/32*4)

HeightMap::HeightMap()
: _heights(NULL)
, _normals(NULL)
, _path(_T(""))
, _map_width(0.0f)
, _map_length(0.0f)
, _map_height_min(0.0f)
, _map_height_max(0.0f)
, _map_width_scale_mpp(1.0f)
, _map_length_scale_mpp(1.0f)
, _map_height_scale_mpp(1.0f)
, _bmp_pixel_width(0)
, _bmp_pixel_length(0)
{
}

HeightMap::~HeightMap()
{
	if (_heights) delete[] _heights;
	if (_normals) delete[] _normals;
}

bool HeightMap::Create(const string_type& path, float width, float length, float height_min, float height_max)
{
	if (path == _T("_PREDEFINE_HM_SLOPE_X_INC"))
		return Create(HM_SLOPE_X_INC, width, length, height_min, height_max);
	else if (path == _T("_PREDEFINE_HM_SLOPE_Y_INC"))
		return Create(HM_SLOPE_X_INC, width, length, height_min, height_max);
	else if (path == _T("_PREDEFINE_HM_SLOPE_XY_INC"))
		return Create(HM_SLOPE_XY_INC, width, length, height_min, height_max);
	else if (path == _T("_PREDEFINE_HM_FLAT"))
		return Create(HM_FLAT, width, length, height_min, height_max);

	_path = path;
	_map_width = width;
	_map_length = length;
	_map_height_min = height_min;
	_map_height_max = height_max;

	FILE* pf = NULL;
	long fsize;
	char* fbuffer = NULL;
	size_t bytesRead;
	int bmp_width_aligned;
	int bmp_byte_per_pixel;
	int bmp_image_size;
	int pindex, pcolor;
	Vector3D v1; v1.Zero();
	Vector3D v2; v2.Zero();
	Vector3D face_n[4];
	Vector3D vertex_n; vertex_n.Zero();

	//--------------------------------------------------------------
	// open bitmap:
	if (0 != _tfopen_s(&pf, _path.c_str(), _T("rb")))
	{
		return false;
	}

	//--------------------------------------------------------------
	// obtain file size:
	fseek (pf , 0 , SEEK_END);
	fsize = ftell(pf);
	rewind (pf);
	
	//--------------------------------------------------------------
	// read bitmap:
	fbuffer = (char*)malloc(sizeof(char)*fsize);
	bytesRead = fread(fbuffer, 1, fsize, pf);
	if (fsize != bytesRead)
		goto __HeightMap_Create_Error;

	fclose(pf); pf = NULL;

	//--------------------------------------------------------------
	// read bitmap file header and bitmap info header:
	BITMAPFILEHEADER* bmfh = (BITMAPFILEHEADER*)fbuffer;
	BITMAPINFOHEADER* bmih = (BITMAPINFOHEADER*)(fbuffer + sizeof(BITMAPFILEHEADER));
	unsigned char* bmp = (unsigned char*)(fbuffer + bmfh->bfOffBits);
	
	assert(bmih->biPlanes == 1 && "HeightMap::Create() : bmih->biPlanes != 1\n");
	//assert(bmih->biBitCount == 8 && "HeightMap::Create() : bmih->biBitCount != 8");
	assert(bmih->biCompression == BI_RGB && "HeightMap::Create() : bmih->biCompression != BI_RGB\n");
	//assert(bmih->biSizeImage == bmih->biWidth*bmih.biHeight && "HeightMap::Create() : bmih->biSizeImage != bmih->biWidth*bmih.biHeight\n");
	assert(bmih->biWidth >= 2 && "HeightMap::Create() : bmih->biWidth < 2\n");
	assert(bmih->biHeight >= 2 && "HeightMap::Create() : bmih->biHeight < 2\n");

	_bmp_pixel_width = bmih->biWidth;
	_bmp_pixel_length = bmih->biHeight;
	bmp_image_size  = _bmp_pixel_width*_bmp_pixel_length;
	
	//--------------------------------------------------------------
	// calculate memory width of one row of bitmap, etc:
	bmp_width_aligned = ALIGNEDWIDTHBYTES(_bmp_pixel_width*bmih->biBitCount);

	switch(bmih->biBitCount)
	{
	case 8:  bmp_byte_per_pixel = 1; break;
	case 24: bmp_byte_per_pixel = 3; break;
	case 32: bmp_byte_per_pixel = 4; break;
	default:
		goto __HeightMap_Create_Error;
	}

	//--------------------------------------------------------------
	// initialize variables from loaded bitmap info:
	_map_width_scale_mpp = _map_width / (float)(_bmp_pixel_width-1);
	_map_length_scale_mpp = _map_length / (float)(_bmp_pixel_length-1);
	_map_height_scale_mpp = (_map_height_max - _map_height_min) / 255.0f;

	//--------------------------------------------------------------
	// allocate memory for vertex heights and vertex normals:
	if (_heights) delete[] _heights;
	_heights = new float[bmp_image_size];
	if (_normals) delete[] _normals;
	_normals = new float[bmp_image_size*3];

	//--------------------------------------------------------------
	// calculate vertex heights:
	for (int py=0; py<_bmp_pixel_length; py++) {
		for (int px=0; px<_bmp_pixel_width; px++) {
			pindex = py*_bmp_pixel_width + px;

			pcolor = 0;
			for (int i=0; i<bmp_byte_per_pixel; i++)
				pcolor += (int)bmp[py*bmp_width_aligned + px*bmp_byte_per_pixel + i];

			_heights[pindex] = _map_height_min + (float)(pcolor) / (float)(bmp_byte_per_pixel) * _map_height_scale_mpp;
		}
	}

	//--------------------------------------------------------------
	// calculate vertex normals:
	_CalculateVertexNormal();

	free(fbuffer);
	
	return true;

__HeightMap_Create_Error:
	if (pf) {
		fclose(pf);
	}
	if (_heights) {
		delete[] _heights;
		_heights = NULL;
	}
	if (_normals) {
		delete[] _normals;
		_normals = NULL;
	}
	if (fbuffer) {
		free(fbuffer);
	}
	return false;
}

bool HeightMap::Create(const ePredefinedHeightMapType hm_type, float width, float length, float height_min, float height_max)
{
	_path = _T("__pre_defined__");
	_map_width = width;
	_map_length = length;
	_map_height_min = height_min;
	_map_height_max = height_max;

	_bmp_pixel_width = PREDEFINE_HBITMAP_WIDTH;
	_bmp_pixel_length = PREDEFINE_HBITMAP_LENGTH;
	int bmp_image_size = _bmp_pixel_width*_bmp_pixel_length;
	int pindex;
	float pcolor;

	//--------------------------------------------------------------
	// initialize variables from loaded bitmap info:
	_map_width_scale_mpp = _map_width / (float)(_bmp_pixel_width - 1);
	_map_length_scale_mpp = _map_length / (float)(_bmp_pixel_length - 1);
	_map_height_scale_mpp = (_map_height_max - _map_height_min) / 255.0f;

	//--------------------------------------------------------------
	// allocate memory for vertex heights and vertex normals:
	if (_heights) delete[] _heights;
	_heights = new float[bmp_image_size];
	if (_normals) delete[] _normals;
	_normals = new float[bmp_image_size * 3];

	//--------------------------------------------------------------
	// calculate vertex heights:
	for (int py = 0; py<_bmp_pixel_length; py++) {
		for (int px = 0; px<_bmp_pixel_width; px++) {
			pindex = py*_bmp_pixel_width + px;

			pcolor = 0.0f;

			switch (hm_type)
			{
			case HM_SLOPE_X_INC:
				pcolor = 255.0f * (float)px / (float)(_bmp_pixel_width-1);
				break;

			case HM_SLOPE_Y_INC:
				pcolor = 255.0f * (float)py / (float)(_bmp_pixel_length - 1);
				break;

			case HM_SLOPE_XY_INC:
				pcolor = 255.0f * (float)(px + py) / (float)(_bmp_pixel_width + _bmp_pixel_length - 2);
				break;

			case HM_FLAT:
			default:
				pcolor = 128;
			}

			_heights[pindex] = _map_height_min + (float)(pcolor) * _map_height_scale_mpp;
		}
	}

	//--------------------------------------------------------------
	// calculate vertex normals:
	_CalculateVertexNormal();

	return true;
/*
__PredefinedHeightMap_Create_Error:
	if (_heights) {
		delete[] _heights;
		_heights = NULL;
	}
	if (_normals) {
		delete[] _normals;
		_normals = NULL;
	}
	return false;
*/
}

void HeightMap::_CalculateVertexNormal()
{
	int pindex;
	Vector3D v1; v1.Zero();
	Vector3D v2; v2.Zero();
	Vector3D face_n[4];
	Vector3D vertex_n; vertex_n.Zero();

	for (int py = 0; py<_bmp_pixel_length; py++) {
		for (int px = 0; px<_bmp_pixel_width; px++) {

			pindex = py*_bmp_pixel_width + px;

			// face normal for the 1st quadrant:
			v1[0] = _map_width_scale_mpp;
			v1[1] = 0.0f;
			v1[2] = (px + 1 < _bmp_pixel_width)
				? (_heights[py*_bmp_pixel_width + (px + 1)] - _heights[pindex])
				: -(_heights[py*_bmp_pixel_width + (px - 1)] - _heights[pindex]);

			v2[0] = 0.0f;
			v2[1] = _map_length_scale_mpp;
			v2[2] = (py + 1 < _bmp_pixel_length)
				? (_heights[(py + 1)*_bmp_pixel_width + px] - _heights[pindex])
				: -(_heights[(py - 1)*_bmp_pixel_width + px] - _heights[pindex]);

			// face_normal = v1 x v2;
			face_n[0] = v1.cross(v2);
			face_n[0].normalize();

			// face normal for the 2nd quadrant:
			v1 = v2;

			v2[0] = -_map_width_scale_mpp;
			v2[1] = 0.0f;
			v2[2] = (px - 1 >= 0)
				? (_heights[py*_bmp_pixel_width + (px - 1)] - _heights[pindex])
				: -(_heights[py*_bmp_pixel_width + (px + 1)] - _heights[pindex]);

			face_n[1] = v1.cross(v2);
			face_n[1].normalize();

			// face normal for the 3rd quadrant:
			v1 = v2;

			v2[0] = 0.0f;
			v2[1] = -_map_length_scale_mpp;
			v2[2] = (py - 1 >= 0)
				? (_heights[(py - 1)*_bmp_pixel_width + px] - _heights[pindex])
				: -(_heights[(py + 1)*_bmp_pixel_width + px] - _heights[pindex]);

			face_n[2] = v1.cross(v2);
			face_n[2].normalize();

			// face normal for the 4th quadrant:
			v1 = v2;

			v2[0] = _map_width_scale_mpp;
			v2[1] = 0.0f;
			v2[2] = (px + 1 < _bmp_pixel_width)
				? (_heights[py*_bmp_pixel_width + (px + 1)] - _heights[pindex])
				: -(_heights[py*_bmp_pixel_width + (px - 1)] - _heights[pindex]);

			face_n[3] = v1.cross(v2);
			face_n[3].normalize();


			vertex_n.Zero();
			for (int i = 0; i<4; i++)
				vertex_n += face_n[i];
			vertex_n.normalize();

			_normals[3 * pindex + 0] = (float)vertex_n[0];
			_normals[3 * pindex + 1] = (float)vertex_n[1];
			_normals[3 * pindex + 2] = (float)vertex_n[2];
		}
	}

}

bool HeightMap::GetHeight(float x, float y, float& height)
{
	// by default, it return "0".
	height = 0.0f;

	if (!_heights) {
		return false;
	}

	int px = (int)((_map_width * 0.5f + x) / _map_width_scale_mpp);
	if (px < 0) px = 0;
	else if (px >= _bmp_pixel_width) px = _bmp_pixel_width-1;
	
	int py = (int)((_map_length * 0.5f + y) / _map_length_scale_mpp);
	if (py < 0) py = 0;
	else if (py >= _bmp_pixel_length) py = _bmp_pixel_length-1;

	// bilinear interpolation:
	float x1 = (float)px * _map_width_scale_mpp - 0.5f * _map_width;
	float y1 = (float)py * _map_length_scale_mpp - 0.5f * _map_length;
	float x2 = x1 + _map_width_scale_mpp;
	float y2 = y1 + _map_length_scale_mpp;
	float Q11 = _heights[py                            *_bmp_pixel_width + px];
	float Q12 = _heights[min(py+1, _bmp_pixel_length-1)*_bmp_pixel_width + px];
	float Q21 = _heights[py                            *_bmp_pixel_width + min(px+1, _bmp_pixel_width-1)];
	float Q22 = _heights[min(py+1, _bmp_pixel_length-1)*_bmp_pixel_width + min(px+1, _bmp_pixel_width-1)];
	height = (Q11 * (x2-x) * (y2-y) + Q21 * (x-x1) * (y2-y) + Q12 * (x2-x) * (y-y1) + Q22 * (x-x1) * (y-y1)) / ((x2-x1) * (y2-y1));

	return true;
}

bool HeightMap::GetNormal(float x, float y, float normal[3])
{
	// by default, it returns "Z" unit vector.
	normal[0] = 0.0f;
	normal[1] = 0.0f;
	normal[2] = 1.0f;

	if (!_heights) {
		return false;
	}

	int px = (int)((_map_width * 0.5f + x) / _map_width_scale_mpp);
	if (px < 0) px = 0;
	else if (px >= _bmp_pixel_width) px = _bmp_pixel_width-1;
	
	int py = (int)((_map_length * 0.5f + y) / _map_length_scale_mpp);
	if (py < 0) py = 0;
	else if (py >= _bmp_pixel_length) py = _bmp_pixel_length-1;

	// bilinear interpolation:
	float x1 = (float)px * _map_width_scale_mpp - 0.5f * _map_width;
	float y1 = (float)py * _map_length_scale_mpp - 0.5f * _map_length;
	float x2 = x1 + _map_width_scale_mpp;
	float y2 = y1 + _map_length_scale_mpp;
	float Q11, Q12, Q21, Q22;
	for (int i=0; i<3; i++) {
		Q11 = _normals[3*(py                            *_bmp_pixel_width + px)                            + i];
		Q12 = _normals[3*(min(py+1, _bmp_pixel_length-1)*_bmp_pixel_width + px)                            + i];
		Q21 = _normals[3*(py                            *_bmp_pixel_width + min(px+1, _bmp_pixel_width-1)) + i];
		Q22 = _normals[3*(min(py+1, _bmp_pixel_length-1)*_bmp_pixel_width + min(px+1, _bmp_pixel_width-1)) + i];
		normal[i] = (Q11 * (x2-x) * (y2-y) + Q21 * (x-x1) * (y2-y) + Q12 * (x2-x) * (y-y1) + Q22 * (x-x1) * (y-y1)) / ((x2-x1) * (y2-y1));
	}

	// normalize:
	float norm = sqrt(pow(normal[0], 2) + pow(normal[1], 2) + pow(normal[2], 2));
	normal[0] /= norm;
	normal[1] /= norm;
	normal[2] /= norm;

	return true;
}

bool HeightMap::GetNormalEx(float x, float y, int wnd_size, float normal[3])
{
	// by default, it returns "Z" unit vector.
	normal[0] = 0.0f;
	normal[1] = 0.0f;
	normal[2] = 1.0f;

	if (!_heights) {
		return false;
	}

	int px = (int)((_map_width * 0.5f + x) / _map_width_scale_mpp);
	if (px < 0) px = 0;
	else if (px >= _bmp_pixel_width) px = _bmp_pixel_width-1;
	
	int py = (int)((_map_length * 0.5f + y) / _map_length_scale_mpp);
	if (py < 0) py = 0;
	else if (py >= _bmp_pixel_length) py = _bmp_pixel_length-1;

	// bilinear interpolation:
	float x1 = (float)px * _map_width_scale_mpp - 0.5f * _map_width;
	float y1 = (float)py * _map_length_scale_mpp - 0.5f * _map_length;
	float x2 = x1 + _map_width_scale_mpp;
	float y2 = y1 + _map_length_scale_mpp;
	float Q11, Q12, Q21, Q22;
	for (int i=0; i<3; i++) {

		Q11 = 0.0f; Q12 = 0.0f; Q21 = 0.0f; Q22 = 0.0f;
		
		for (int r=0; r<wnd_size; r++) {
			for (int c=0; c<wnd_size; c++) {
				Q11 += _normals[3*(max(py-r, 0)                    *_bmp_pixel_width + max(px-c, 0))                    + i];
				Q12 += _normals[3*(min(py+1+r, _bmp_pixel_length-1)*_bmp_pixel_width + max(px-c, 0))                    + i];
				Q21 += _normals[3*(max(py-r, 0)                    *_bmp_pixel_width + min(px+1+c, _bmp_pixel_width-1)) + i];
				Q22 += _normals[3*(min(py+1+r, _bmp_pixel_length-1)*_bmp_pixel_width + min(px+1+c, _bmp_pixel_width-1)) + i];
			}
		}
		
		normal[i] = (Q11 * (x2-x) * (y2-y) + Q21 * (x-x1) * (y2-y) + Q12 * (x2-x) * (y-y1) + Q22 * (x-x1) * (y-y1)) / ((x2-x1) * (y2-y1));
	}

	// normalize:
	float norm = sqrt(pow(normal[0], 2) + pow(normal[1], 2) + pow(normal[2], 2));
	normal[0] /= norm;
	normal[1] /= norm;
	normal[2] /= norm;

	return true;
}
