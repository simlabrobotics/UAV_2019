/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#include "stdafx.h"
#include "rLabServer.h"

static void dumpMessage(kaiMsg& msg, const char* dec)
{
	float arg[4];
	memcpy(arg, (char*)msg.buffer() + 8, sizeof(float)*4);
	printf("%s[id=%02d] %.3f\t %.3f\t %.3f\t %.3f\n",
		dec, msg.id(), arg[0], arg[1], arg[2], arg[3]);
}

rLabServer::rLabServer()
	: kaiServer()
	, _recv_count(0)
	, _send_count(0)
{
}

void rLabServer::onMessage(int id, kaiMsg &msg)
{
	if ((++_recv_count) % 1000 == 0)
	{
		printf("rLabServer> %d commands received.\n", _recv_count);
	}

	dumpMessage(msg, "<< ");
}

void rLabServer::onAccept(int id)
{
	kaiSocket* client = findClient(id);
	if (client)
	{
		printf(">> New controller(%d) is connected.\n", id);
	}
}

void rLabServer::onClose(int id)
{
	kaiSocket* client = findClient(id);
	if (client)
	{
		printf(">> Controller(%d) is disconnected.\n", id);
	}
}
