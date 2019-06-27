/* RoboticsLab, Copyright 2008-2016 Wonik Robotics. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics.
 */
// app_UAVClient.cpp : Defines the entry point for the console application.
//
#include "ClientSocket.h"
#include <rlab/math/rMath.h>

#define RD_BOUND(val, lbound, ubound) \
	((val) < (lbound) ? (lbound) : ((val) > (ubound) ? (ubound) : (val)))

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
float v_des = 0.0f; // desired velocity
float theta_des = 0.0f; // desired steer angle
const float V_INC = 0.27f;//0.01;
const float THETA_INC = (float)(5.0*DEGREE);
float pto_des = 0.0f;
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

char* serverIP = "127.0.0.1";
unsigned short serverPort = 5251;

int main(int argc, char *argv[])
{
	if (argc > 1) {
		serverIP = argv[1];
	}
	if (argc > 2) {
		serverPort = (unsigned short)atoi(argv[2]);
	}

	kaiInitialize();

	// create client socket
	CClientSocket client;
	client.create(true, kaiON_MESSAGE | kaiON_SEND);
	// connect to the server.
	if (kaiSUCCESS != client.connect(serverIP, serverPort))
	{
		printf("ERROR: FAIL TO CONNECT TO SIMULATOR !\n");
		goto __finish;
	}

	printf("*****************INSTRUCTION*****************\n");
	printf("\'Q\' or \'q\': quit this program\n");
	printf("\'W\' or \'w\': increase velocity\n");
	printf("\'X\' or \'x\': decrease velocity\n");
	printf("\'A\' or \'a\': increase angular velocity in CCW direction\n");
	printf("\'D\' or \'d\': increase angular velocity in CW direction\n");
	printf("\'S\' or \'s\': stop\n");
	printf("\n");
	printf("\'R\' or \'r\': lift up PTO\n");
	printf("\'F\' or \'f\': let down PTO\n");
	printf("\n");
	printf("\'P\' or \'p\': reset Path\n");
	printf("---------------------------------------------\n");
	printf("\'C\' or \'c\': request coverage statistics\n");
	printf("\'I\' or \'i\': request indicators\n");
	printf("*********************************************\n");

	bool bRun = true;
	while (bRun) {
		if (!_kbhit()) {
			/// send data here
			client.sendVelocity(v_des, theta_des);
			client.recv();
			Sleep(5);
		}
		else {
			int c = _getch();
			switch (c) {
				// move UAV
				case 'w': case 'W':
					v_des += V_INC;
					break;
				case 'x': case 'X':
					v_des -= V_INC;
					break;
				case 'a': case 'A':
					theta_des += THETA_INC;
					break;
				case 'd': case 'D':
					theta_des -= THETA_INC;
					break;
				case 's': case 'S':
					v_des = 0;
					break;

				// up&down worker
				case 'r': case 'R':
					client.sendMovePTO(0);
					break;
				case 'f': case 'F':
					client.sendMovePTO(1);
					break;

				// reset path
				case 'p': case 'P':
					client.resetPath();
					break;

				// request coverage data
				case 'c': case 'C':
					client.reqCoverage();
					break;

				// request other indicators
				case 'i': case 'I':
					client.reqIndicators();
					break;

				case 'q': case 'Q':
					bRun = false;
					break;
			}
		}
	}

__finish:
	client.close();
	kaiShutdown();

	return 0;
}
