/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#ifndef __PERLIN_H__
#define __PERLIN_H__


enum ePerlinInterpolator
{
	ePI_LINEAR,
	ePI_COSINE,
	ePI_CUBIC,
	ePI_COUNT
};

typedef void (*fnCallbackPerlinProgress)(int);

enum ePerlinError
{
	eP_SUCCESS,
	eP_ERROR_UNKNOWN,
	eP_ERROR_FILEOPEN,
};

int GenPerlin(int width, int height, float persistence, ePerlinInterpolator interpolator, int octave, bool smoothing, char const * filename, fnCallbackPerlinProgress callback);



#endif // __PERLIN_H__