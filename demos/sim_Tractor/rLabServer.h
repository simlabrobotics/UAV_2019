/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#ifndef __RLABSERVER_H__
#define __RLABSERVER_H__

#include "kai.h"

class rLabServer : public kaiServer
{
public:
	rLabServer();
	virtual void onAccept(int id);
	virtual void onClose(int id);
	virtual void onMessage(int id, kaiMsg &msg);

private:
	int _loop_count;
	int _recv_count;
	int _send_count;
};

#endif // __RLABSERVER_H__
