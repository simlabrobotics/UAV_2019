/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
#include "stdafx.h"
#include "rLabServer.h"

static void dumpMessage(kaiMsg& msg, const char* dec)
{
	return;

	float arg[4];
	memcpy(arg, (char*)msg.buffer() + Size_kaiHEADER, sizeof(float)*4);
	printf("%s[id=%02d]", dec, msg.id());
	for (int i = 0; i < 16; i++) {
		printf(" %02x", *(msg.buffer() + Size_kaiHEADER + i));
		if ((i % 4) == 3)
			printf(" ");
	}
	printf("\tflot[] = %.3f %.3f %.3f %.3f", arg[0], arg[1], arg[2], arg[3]);
	printf("\n");
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
