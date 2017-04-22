#include "./pixy.c"
#include "./auton.c"

Pixy robotPixy;
PixyBlock* largestBlock;
PixyBlock* largestBlockInClaw;
int largestBlockX; int largestBlockY;

int autoQuit = 0;

task quitAuto
{
	if (vexRT[Btn7R])
	{
		autoQuit = 1;
	}
}

task PixyPackets
{
	newPixy(&robotPixy, UART2, baudRate38400);

	while (1==1)
	{
		pixyUpdate(&robotPixy);
		largestBlock = pixyGetLargestBlock(&robotPixy, 1);
		//largestBlockInClaw = pixyGetLargestBlockInClaw(&robotPixy, 1);
		if (largestBlock != 0) largestBlockX = largestBlock->x;
		if (largestBlock != 0) largestBlockY = largestBlock->y;
		//pixyPrint(&robotPixy);

		/*if (largestBlock != 0)
		pixyPrintBlock(largestBlock);
		else
		writeDebugStreamLine("---");*/

		delay(50);
	}
}

const int PIXY_MID = 170;

//turns in place to face an object
void PixyTurn(float cutoff = 30)
{
	float error = 0;
	error = largestBlockX - PIXY_MID;

	int dir = sgn(error);

	masterSetTurn(dir * 127);

	while (true)
	{
		error = largestBlockX - PIXY_MID;

		if (abs(error) <= cutoff)
		{
			masterSetDrive(-5 * dir, 5 * dir);
			break;
		}
		//if (abs(gyroYawRate) < 0.3) break;
		if (autoQuit == 1) break;
		delay(50);
	}
}

void PixyScore()
{
	masterSetDrive(100);
	wait1Msec(2000);
	masterSetDrive(0);
	masterSetClaw(OPENED);
}

void PixyBackFromFence()
{
	masterSetDrive(-100);
	wait1Msec(2000);
	masterSetDrive(0);
}

int directionFacing = 0;
void PixyAuton()
{
	while(autoQuit == 0)
	{
		delay(100);
		while (largestBlock == 0  && autoQuit == 0)
		{
			for (int i = 0; i < 5; i++)
			{
				wait1Msec(50);
				if (largestBlock != 0) break;
			}
			if (largestBlock != 0) break;
			PixyTurn();
			masterSetClaw(0);
			wait1Msec(200);
			masterSetDrive(100);
			wait1Msec(2000);
			masterSetClaw(1);
			masterSetDrive(0);
			wait1Msec(200);
			masterSetDrive(-70);
			wait1Msec(600);
			//turn to face fence
			masterSetDrive(-70);
			wait1Msec(600);
			masterSetLift(UP);
			sleep(1000);
			PixyScore();
			PixyBackFromFence();
			if (directionFacing == 1) directionFacing = -1;
			else if (directionFacing == -1) directionFacing = 1;
		}
	}
}
