#ifndef __UAV_DEF_H__
#define __UAV_DEF_H__


/////////////////////////////////////////////////////////
#pragma pack(push)
#pragma pack(1)
/////////////////////////////////////////////////////////

typedef struct sMovingPTO
{
	int				signal;
	float			reserved_0;
	float			reserved_1;
	float			reserved_2;
} PTO;

typedef struct sDownWorker
{
	float			down;
	float			reserved_0;
	float			reserved_1;
	float			reserved_2;
} DOWNWORKER;

typedef struct sUpWorker
{
	float			up;
	float			reserved_0;
	float			reserved_1;
	float			reserved_2;
} UPWORKER;

typedef struct sWidth
{
	float			width;
	float			reserved_0;
	float			reserved_1;
	float			reserved_2;
} PTOWIDTH;

typedef struct sNetData
{
	unsigned short	id;
	unsigned short	size;
	short			resereved;
	unsigned char	flag;
	char			checksum;
	float data[4];
} DATAFORMAT;

typedef struct sWayPoint
{
	unsigned int	index;
	float			x;
	float			y;
	float			angle;
} WAYPOINT;

typedef struct sPose2D
{
	float			x;
	float			y;
	float			heading;
	float			reserved;
} POSE2D;

typedef struct sMotionState
{
	float			heading;
	float			velocity;
	float			reserved_0;
	float			reserved_1;
} MOTIONSTATE;

typedef struct sWorkCoverage
{
	int				total;
	int				worked;
	float			reserved_0;
	float			reserved_1;
} WORKCOVERAGE;

typedef struct sParameter
{
	int				type;
	float			param1;
	float			param2;
	float			param3;
} PARAMETER;

enum eParamType
{
	// differential-drive vehicle(combine, etc):
	PTYPE_lateral_resistance = 1,
	PTYPE_gravitational_acceleration = 2,
	PTYPE_slip_ratio_mean_and_std = 3,
	PTYPE_maximum_wheel_angular_velocity = 4,
	PTYPE_maximum_wheel_angular_acceleration = 5,
	PTYPE_velocity_threshold_to_apply_slippage = 6,
	PTYPE_wheel_velocity_noise_mean_and_std = 7,
	PTYPE_deformation_modulus = 8,
	PTYPE_internal_shearing_resistance = 9,
	PTYPE_cohesive_modulus_of_terrain_deformation = 10,
	PTYPE_frictional_modulus_of_terrain_deformation = 11,
	PTYPE_exponent_of_terrain_deformation = 12,
	PTYPE_tread_length = 101,
	PTYPE_track_length = 102,
	PTYPE_track_width = 103,
	PTYPE_vehicle_mass = 104,
	PTYPE_wheel_radius = 105,
	// car-like vehicle(tractor, etc):
	PTYPE_C_vehicle_mass = 257,
	PTYPE_C_vehicle_moment_of_inertia_at_COG = 258,
	PTYPE_C_gravitational_acceleration = 259,
	PTYPE_C_slip_ratio_mean_and_std = 260,
	PTYPE_C_maximum_velocity = 261,
	PTYPE_C_maximum_acceleration = 262,
	PTYPE_C_maximum_steer_angle = 263,
	PTYPE_C_maximum_steer_velocity = 264,
	PTYPE_C_steer_ratio = 265,
	PTYPE_C_velocity_noise_mean_and_std = 266,
	PTYPE_C_velocity_threshold_to_apply_slippage = 271,
	PTYPE_C_Cf = 272,
	PTYPE_C_Cr = 273,
	PTYPE_C_Lf = 274,
	PTYPE_C_Lr = 275,
	PTYPE_C_velocity_threshold_to_apply_slippage_due_to_slope = 281,
	PTYPE_C_tire_radius = 282,
	PTYPE_C_tire_contact_length = 283,
	PTYPE_C_tire_contact_width = 284,
	PTYPE_C_soil_adhesiveness = 285,
	PTYPE_C_internal_shearing_resistance = 286,
	PTYPE_C_shear_modulus_of_elasticity = 287,
	PTYPE_C_cohesive_modulus_of_terrain_deformation = 288,
	PTYPE_C_frictional_modulus_of_terrain_deformation = 289,
	PTYPE_C_exponent_of_terrain_deformation = 290,
	PTYPE_C_slope_slip_ratio_mean_and_std = 291,
	PTYPE_C_ratio_of_terrain_deformation = 292,
};

#define HMAP_SIZE_MAX (1024*1024)

typedef struct sHeightMap
{
	float			heights[HMAP_SIZE_MAX];
	float			normals[HMAP_SIZE_MAX*3];
	float			map_width;
	float			map_length;
	float			map_height_min;
	float			map_height_max;
	float			map_width_scale;
	float			map_length_scale;
	float			map_height_scale;
	unsigned char	bmp[HMAP_SIZE_MAX];
	int				bmp_pixel_width;
	int				bmp_pixel_length;
} HMAP_DATA;

/////////////////////////////////////////////////////////
#pragma pack(pop)
/////////////////////////////////////////////////////////

#endif // __UAV_DEF_H__
