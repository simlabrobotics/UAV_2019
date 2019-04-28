#ifndef __UAV_PROTOCOL_H__
#define __UAV_PROTOCOL_H__

/**
 PAYLOAD is always consist of 4 float variables.
 Unused data slots are filled with zero.

 ----------------------------------------------------
 |	arg1	|	arg2	|	arg3	|	arg4		|
 |	float	|	float	|	float	|	float		|
 ----------------------------------------------------
*/


/******************************************************************************
 Add a new waypoint.
 @arg1 waypoint index
 @arg2 x coordinate in meter
 @arg3 y coordinate in meter
 @arg4 heading angle in degree
 */
#define UAVP_ADD_WAYPOINT				(0)


/******************************************************************************
 Set target velocity of the wheels periodically.
 @arg1 left wheel velocity in rad/sec
 @arg2 right wheel velocity in rad/sec
*/
#define UAVP_SET_WHEEL_VELOCITY			(1)
/**
 Notified with vehicle pose periodically.
 @arg1 heading angle in degree
 @arg2 x coordinate in global frame in meter
 @arg2 y coordinate in global frame in meter
 @arg2 z coordinate in global frame in meter
*/
#define UAVP_GET_SENSORS				(2)


/******************************************************************************
 Set tool(cutter) position.
 @arg1 {1:put it down}, {0:lift it up}
*/
#define UAVP_MOVE_PTO					(3)


/******************************************************************************
 */
#define UAVP_MOTION_STATE				(4)
//#define UAVP_UP_WORKER					(5)
//#define UAVP_DOWN_WORKER				(6)
//#define UAVP_PTO_WIDTH					(7)


/******************************************************************************
 Request working coverage statistics.
 */
#define UAVP_REQ_COVERAGE				(8)
/**
 Acknowledge working coverage statistics.
 @arg1 total cell count
 @arg2 cell count occupied
 */
#define UAVP_ACK_COVERAGE				(9)


/******************************************************************************
 Request slip angle data
 */
#define UAVP_REQ_SLIP_ANGLE				(10)
/**
 Acknowledge slip angle data
 @arg1 COG slip angle in degree (alpha)
 @arg2 slip ratio of left track (i_L)
 @arg3 slip ratio of right track (i_R)
 */
#define UAVP_ACK_SLIP_ANGLE				(11)


//#define UAVP_REQ_LATERAL_DISTURBANCE	(12)
//#define UAVP_ACK_LATERAL_DISTURBANCE	(13)


/******************************************************************************
 Request GPS sensor data.
 */
#define UAVP_REQ_GPS					(14)
/**
 Acknowledge GPS sensor data.
 @arg1 x coordinate in global coordinate frame in meter
 @arg2 y coordinate in global coordinate frame in meter
 @arg3 z coordinate in global coordinate frame in meter
 */
#define UAVP_ACK_GPS					(15)


/******************************************************************************
 Request IMU sensor data.
 */
#define UAVP_REQ_IMU					(16)
/**
 Acknowledge IMU sensor data.
 @arg1 yaw angle in degree
 @arg2 pitch angle in degree
 @arg3 roll angle in degree
 */
#define UAVP_ACK_IMU					(17)


/******************************************************************************
 Request vehicle status.
 */
#define UAVP_REQ_VEHICLE_STATUS			(18)
/**
 Acknowledge vehicle status.
 @arg1 left wheel velocity in rad/sec
 @arg2 right wheel velocity in rad/sec
 @arg3 linear velocity of the vehicle in m/s
 @arg4 angular velocity of the vehicle in rad/sec
 */
#define UAVP_ACK_VEHICLE_STATUS			(19)


/******************************************************************************
 Set maximum wheel velocity.
 @arg1 maximum left wheel velocity in rad/sec
 @arg2 maximum right wheel velocity in rad/sec
*/
#define UAVP_SET_MAXIMUM_WHEEL_VELOCITY	(20)


/******************************************************************************
 Set vehicle pose.
 @arg1 x coordinate in meter
 @arg2 y coordinate in meter
 @arg3 heading angle in degree
*/
#define UAVP_SET_VEHICLE_POSE			(22)


/******************************************************************************
Set target velocity of the vehicle.
@arg1 target linear velocity of the vehicle in the forward direction in m/sec
@arg2 target rotational velocity of the vehicle in rad/sec
*/
#define UAVP_SET_VELOCITY				(24)


/******************************************************************************
Set maximum velocity of the vehicle.
@arg1 maximum linear velocity of the vehicle in the forward direction in m/sec
@arg2 maximum rotational velocity of the vehicle in rad/sec
*/
#define UAVP_SET_MAXIMUM_VELOCITY		(26)


/******************************************************************************
 Set parameter
 @arg1 parameter type
	1: lateral resistance
	2: gravitational acceleration in m/s^2
	3: slip ratio mean and std
	4: maximum wheel angular velocity in m/s
	5: maximum wheel angular acceleration m/s^2
	6: velocity threshold to apply slippage in m/s
	7: wheel velocity noise mean and std
	8: deformation modulus
	9: internal shearing resistance
	10: cohesive modulus of terrain deformation
	11: frictional modulus of terrain deformation
	12: exponent of terrain deformation
	101: tread length in meter
	102: track length in meter
	103: track width in meter
	104: vehicle mass in kg
	105: wheel(sprocket) radius
 @arg2 parameter value 1
 @arg3 parameter value 2
 @arg4 parameter value 3
 */
#define UAVP_SET_PARAMETER				(100)


/******************************************************************************
 Request parameter
 @arg1 parameter type (see definition of UAVP_SET_PARAMETER)
 */
#define UAVP_REQ_PARAMETER				(102)
/******************************************************************************
 Acknowledge current parameter value
 @arg1 parameter type (see definition of UAVP_SET_PARAMETER)
 @arg2 parameter value 1
 @arg3 parameter value 2
 @arg4 parameter value 3
 */
#define UAVP_ACK_PARAMETER				(103)


/******************************************************************************
 Request indicator values
 */
#define UAVP_REQ_INDICATORS				(104)
/**
 Acknowledge indicators.
 @arg1 tractive force of left track in N
 @arg2 tractive force of right track in N
 @arg3 sinkage in meter
 @arg4 motion resistance due to compaction in kN
*/
#define UAVP_ACK_INDICATORS				(105)



#endif // __UAV_PROTOCOL_H__
