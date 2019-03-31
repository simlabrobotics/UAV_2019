#ifndef __UAV_HMAP_H__
#define __UAV_HMAP_H__

#include "UAV_def.h"

static HANDLE s_hShm = NULL;
static HMAP_DATA s_shm = NULL;

inline HMAP_DATA openHeightMap()
{
	bool bCreated = false;

	if (s_shm != NULL)
		return s_shm;
		
	s_hShm = OpenFileMapping(
		FILE_MAP_ALL_ACCESS, 
		FALSE, 
		TEXT("UAV::HeightMap"));
	if (s_hShm == NULL)
	{
		s_hShm = CreateFileMapping(
			INVALID_HANDLE_VALUE,
			NULL,
			PAGE_READWRITE,
			0,
			sizeof(HMAP_DATA),
			TEXT("UAV::HeightMap"));
		if (s_hShm == NULL)
			return NULL;

		bCreated = true;
	}
	
	s_shm = (PROBOT_DATA)MapViewOfFile(
		s_hShm, 
		FILE_MAP_ALL_ACCESS, 
		0, 0, 0);

	if (!s_shm)
	{
		printf("\n--------------------------------------------------------------------\n");
		printf("[ERROR] Could not get or create height map.\n");
		CloseHandle(s_hShm);
		s_shm = NULL;
		s_hShm = NULL;
	}

	if (bCreated && s_shm)
		memset(s_shm, 0, sizeof(HMAP_DATA));

	return s_shm;
}

inline void closeHeightMap()
{
	if (s_hShm)
		CloseHandle(s_hShm);
	s_shm = NULL;
	s_hShm = NULL;
}

#endif // __UAV_HMAP_H__
