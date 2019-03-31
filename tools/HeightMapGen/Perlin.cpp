/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#include "stdafx.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "Perlin.h"

int _width = 0;
int _height = 0;
int _widthAligned = 0;
double* _noise = NULL;
double* _perlin = NULL;

#define ALIGNEDWIDTHBYTES(bits) (((bits)+31)/32*4)

double Power(double d, int pwr)
{
	if (pwr <= 0)
		return 1.0;
	else
		return d * Power(d, pwr-1);
}

int Power(int d, int pwr)
{
	if (pwr <= 0)
		return 1;
	else
		return d * Power(d, pwr-1);
}

double MyRand(int n) // returns a random number between -1.0 and 1.0
{
	n = Power((n << 13), n);
	return (1.0 - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0);
}

double Noise(int x, int y)
{
	if (x < 0) x = 0;
	if (x >= _width) x = _width -1;
	if (y < 0) y = 0;
	if (y >= _height) x = _height -1;

	return _noise[y*_width+x];
}

double Interpolate_Linear(double a, double b, double f)
{
	return a*(1-f) + b*f;
}

double Interpolate_Cosine(double a, double b, double f)
{
	double ft = f * 3.1415927;
	double f2 = (1 - cos(ft)) * 0.5;
	return a*(1-f2) + b*f2;
}

double Interpolate_Linear_2D(int x, int y, int freq)
{
	int dx = (int)(_width / freq);
	int dy = (int)(_height / freq);
	int x0 = (int)(x / dx) * dx;
	int x1 = min(x0 + dx, _width);
	int y0 = (int)(y / dy) * dy;
	int y1 = min(y0 + dy, _height);
	double x_frag = (double)(x - x0) / (x1 - x0);
	double y_frag = (double)(y - y0) / (y1 - y0);
	double v0 = Interpolate_Linear(_noise[y0*(_width+1)+x0], _noise[y0*(_width+1)+x1], x_frag);
	double v1 = Interpolate_Linear(_noise[y1*(_width+1)+x0], _noise[y1*(_width+1)+x1], x_frag);
	return Interpolate_Linear(v0, v1, y_frag);
}

double Interpolate_Cosine_2D(int x, int y, int freq)
{
	int dx = (int)(_width / freq);
	int dy = (int)(_height / freq);
	int x0 = (int)(x / dx) * dx;
	int x1 = min(x0 + dx, _width);
	int y0 = (int)(y / dy) * dy;
	int y1 = min(y0 + dy, _height);
	double x_frag = (double)(x - x0) / (x1 - x0);
	double y_frag = (double)(y - y0) / (y1 - y0);
	double v0 = Interpolate_Cosine(_noise[y0*(_width+1)+x0], _noise[y0*(_width+1)+x1], x_frag);
	double v1 = Interpolate_Cosine(_noise[y1*(_width+1)+x0], _noise[y1*(_width+1)+x1], x_frag);
	return Interpolate_Cosine(v0, v1, y_frag);
}

double Interpolate_Cubic_2D(int x, int y, int freq)
{
	return 0.0;
}

double Smooth(int x, int y)
{
	double corners = (Noise(x-1, y-1) + Noise(x+1, y-1) + Noise(x-1, y+1) + Noise(x+1, y+1)) / 16;
	double sides   = (Noise(x  , y-1) + Noise(x  , y+1) + Noise(x-1, y  ) + Noise(x+1, y  )) / 8;
	double center  = Noise(x, y) / 4;
	return corners + sides + center;
}

void Perlin_Nth(int n)
{
	if (n >= _width || n >= _height)
		return;

	for (int j=0; j<_height; j++)
	{
		for (int i=0; i<_width; i++)
		{
		}
	}

	Perlin_Nth(n+1);
}

int GenPerlin(int width, int height, float persistence, ePerlinInterpolator interpolator, int octave, bool smoothing, char const * filename, fnCallbackPerlinProgress callback)
{
	int rtn = eP_SUCCESS;
	FILE *pFile = NULL;
	unsigned char* bitmap = NULL;

	if (width <= 0 
		|| height <= 0 
		|| persistence <= 1.0e-5 
		|| interpolator >= ePI_COUNT || interpolator < 0
		|| octave < 1 || octave > 10 
		|| filename == NULL || filename[0] == 0
		)
	{
		rtn = eP_ERROR_UNKNOWN;
		goto perlin_clean;
	}
	
	_width = width;
	_height = height;
	_widthAligned = ALIGNEDWIDTHBYTES(_width*8);
	_noise = new double[(width+1)*(height+1)];
	_perlin = new double[width*height];
	int frequency = 0;
	double amplitude = 0.0;
	double scale = 0.0;
	bitmap = new unsigned char[_widthAligned*height];

	if (callback)
		callback(20);

	memset(_perlin, 0, sizeof(double)*width*height);

	// generate random noise
	srand((unsigned)time(NULL));
	for (int i=0; i<(width+1)*(height+1); i++)
	{
		_noise[i] = (double)rand() / (RAND_MAX);
		//_noise[i] = MyRand(i);
	}

	// smooth noise
	if (smoothing)
	{
		double* noise_smoothed = new double[(width+1)*(height+1)];
		memcpy(noise_smoothed, _noise, sizeof(double)*(width+1)*(height+0));
		for (int j=1; j<height; j++)
		{
			for (int i=1; i<width; i++)
			{
				noise_smoothed[j*(width+1)+i] = Smooth(i, j);
			}
		}
		memcpy(_noise, noise_smoothed, sizeof(double)*(width+1)*(height+0));
		delete [] noise_smoothed;
	}

	if (callback)
		callback(40);

	for (int k=0; k<octave; k++)
	{
		frequency = Power(2, k);
		amplitude = Power(persistence, k);
		scale += amplitude;

		if (frequency > width || frequency > height)
			continue;
		
		for (int j=0; j<height; j++)
		{
			for (int i=0; i<width; i++)
			{
				switch (interpolator)
				{
				case ePI_LINEAR:
					_perlin[width*j+i] += amplitude * Interpolate_Linear_2D(i, j, frequency);
					break;
				
				case ePI_CUBIC:
				case ePI_COSINE:
					_perlin[width*j+i] += amplitude * Interpolate_Cosine_2D(i, j, frequency);
					break;
				}
			}
		}
	}

	for (int j=0; j<height; j++)
	{
		for (int i=0; i<width; i++)
		{
			bitmap[j*_widthAligned+i] = (unsigned char)((_perlin[j*width+i] / scale) * 255);
		}
	}

	if (callback)
		callback(80);

	//Create a new file for writing
	if (0 != fopen_s(&pFile, filename, "wb"))
	{
		rtn = eP_ERROR_FILEOPEN;
		goto perlin_clean;
	}
	
	RGBQUAD bmiColors[256];
	for (int i=0; i<256; i++)
	{
		bmiColors[i].rgbRed      = (BYTE)i;
		bmiColors[i].rgbGreen    = (BYTE)i;
		bmiColors[i].rgbBlue     = (BYTE)i;
		bmiColors[i].rgbReserved = 0x00;
	}

	BITMAPINFOHEADER bmih;
	bmih.biSize = sizeof(BITMAPINFOHEADER);
	bmih.biWidth = width;
	bmih.biHeight = height;
	bmih.biPlanes = 1;
	bmih.biBitCount = 8;
	bmih.biCompression = BI_RGB;
	bmih.biSizeImage = _widthAligned*height;
	bmih.biXPelsPerMeter = 1;
	bmih.biYPelsPerMeter = 1;
	bmih.biClrUsed = 0;
	bmih.biClrImportant = 0;

	BITMAPFILEHEADER bmfh;
	int nBitsOffset = sizeof(BITMAPFILEHEADER) + bmih.biSize + sizeof(bmiColors);
	long lImageSize = bmih.biSizeImage;
	long lFileSize = nBitsOffset + lImageSize;
	bmfh.bfType = 'B' + ('M'<<8);
	bmfh.bfOffBits = nBitsOffset;
	bmfh.bfSize = lFileSize;
	bmfh.bfReserved1 = bmfh.bfReserved2 = 0;

	//Write "BITMAPFILEHEADER"
	UINT nWrittenFileHeaderSize = fwrite(&bmfh, 1, sizeof(BITMAPFILEHEADER), pFile);
	//followed by "BITMAPINFOHEADER"
	UINT nWrittenInfoHeaderSize = fwrite(&bmih, 1, sizeof(BITMAPINFOHEADER), pFile);
	//followed by "RGBQUAD array"
	UINT nWrittenRGBQuadsSize = fwrite(bmiColors, 1, sizeof(bmiColors), pFile);
	//Finally, write the image data itself 
	UINT nWrittenDIBDataSize = fwrite(bitmap, 1, lImageSize, pFile);

	if (callback)
		callback(100);

perlin_clean:
	if (pFile)
		fclose(pFile);
	delete [] _noise;
	delete [] _perlin;
	_width = 0;
	_height = 0;
	delete [] bitmap;

	return rtn;
}