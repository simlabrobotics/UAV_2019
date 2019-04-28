/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#ifndef __UAV_CMD_H__
#define __UAV_CMD_H__

#include <rlab/command/rCmdMobile.h>

// device commands
#define UAVDRV_DATAPORT_SET_POSE					(255)	// set (x, y, th)
#define UAVDRV_DATAPORT_SET_VEL_IMMEDIATE			(254)	// set (v_des, thdot_des)
#define UAVDRV_DATAPORT_SET_VEL_TARGET				(253)	// set (v_des, thdot_des)
#define UAVDRV_DATAPORT_SET_WHEEL_VEL_IMMEDIATE		(252)	// set (wL_des, wR_des)
#define UAVDRV_DATAPORT_SET_WHEEL_VEL_TARGET		(251)	// set (wL_des, wR_des)
#define UAVDRV_DATAPORT_SET_MAXIMUM_VEL				(250)
#define UAVDRV_DATAPORT_SET_MAXIMUM_WHEEL_VEL		(250)
#define UAVDRV_DATAPORT_SET_PARAMETER				(249)

// port definition for monitoring UAV drive devices
#define UAVDRV_MONITORPORT_POSE						(0)
#define UAVDRV_MONITORPORT_POSE_LOCAL				(1)
#define UAVDRV_MONITORPORT_VELOCITY					(2)		// get (gxdot, gydot, gthdot)
#define UAVDRV_MONITORPORT_VELOCITY_LOCAL			(3)		// get (v, thdot)
#define UAVDRV_MONITORPORT_VELOCITY_LOCAL_WO_SLIP	(4)
#define UAVDRV_MONITORPORT_WHEEL_VELOCITY			(7)		// get (wL, wR)
#define UAVDRV_MONITORPORT_WHEEL_VELOCITY_WO_SLIP	(8)
#define UAVDRV_MONITORPORT_SLIP_ANGLE				(11)
#define UAVDRV_MONITORPORT_LATERAL_FORCE			(12)
#define UAVDRV_MONITORPORT_TRACTIVE_FORCE			(13)
#define UAVDRV_MONITORPORT_SINKAGE					(14)
#define UAVDRV_MONITORPORT_MOTION_RESISTANCE		(15)
#define UAVDRV_MONITORPORT_PARAMETER				(16)


#endif // __UAV_CMD_H__
