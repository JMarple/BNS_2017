const int PIXY_MID = 170;

int PickHeading(int oldColumn, PixyBlock* targetBlock)
{
	int xValue, yValue, newColumn = 0;

	xValue = targetBlock->x;
	yValue = targetBlock->y;

	if (xValue < (-6/45)*yValue + 133.)
	{
		newColumn = oldColumn - 2;
	}
	else if (xValue < (-2/45)*yValue + 171.)
	{
		newColumn = oldColumn - 1;
	}
	else if (xValue < (2/45)*yValue + 203.)
	{
		newColumn = oldColumn;
	}
	else if (xValue < (6/45)*yValue + 227.)
	{
		newColumn = oldColumn + 1;
	}
	else
	{
		newColumn = oldColumn + 2;
	}

	return newColumn;
}

//turns in place to face an object, returns the new angle
float PixyTurn(float cutoff = 3)
{
	struct PID pixyHeadingPID;
	PIDInit(&pixyHeadingPID, 0.4, 0, 0);

	while (1==1)
	{
		float error = 0;
		error = largestBlockX - PIXY_MID;

		float pidResult = PIDUpdate(&pixyHeadingPID, error, 1);

		if(error >= 0)
		{
			leftDrive(pidResult);
			rightDrive(-pidResult);
		}
		else
		{
			leftDrive(-pidResult);
			rightDrive(pidResult);
		}

		if (abs(error) <= cutoff) break;
		if (abs(error) <= 50 && abs(gyroYawRate) < 0.3) break;
		delay(50);
	}
	//however much it turned, assume it was the right amount for the purposes of correcting
	return GyroGetAngle();
}

void ColumnTransition(int oldColumn, int newColumn, PixyBlock* targetBlock)
{
	float columnDist = 500; //TODO: figure out what this number should be

	float oldHeading = GyroGetAngle();
	float tempHeading = PixyTurn();

	float driveDist = (1 / sinDegrees(fabs(oldHeading - tempHeading)) ) * columnDist;

	driveHoldHeading(driveDist, 127, tempHeading, DRIVE_ENCODERS, ACCEL_FAST, 50, 4);

	driveTurnInPlace(oldHeading, PID_INPLACE_TURN_SMALL_NORMAL);
}

void PixyScore(float currentHeading)
{
	driveHoldHeading(550, 127, currentHeading, DRIVE_ENCODERS, ACCEL_FAST, 50, 4);
	driveHoldHeading(550, 127, currentHeading, DRIVE_LINES, ACCEL_NONE, 127, 4);
	setClaw(0);
	drivePower(127);
	wait1Msec(100);
	drivePower(50);
	wait1Msec(250);
}

float PixyBackFromFence(int dir = 1, float currentHeading)
{
	driveHoldHeading(-300, -70, currentHeading, DRIVE_ENCODERS, ACCEL_SLOW, -40);
	SetLiftHeight(LIFT_LOW_HEIGHT);
	driveHoldHeading(-700, -70, currentHeading, DRIVE_LINES);
	brake(-1);

	currentHeading += ROTATE_RIGHT * sgn(dir);
	driveTurnInPlace(currentHeading, PID_INPLACE_TURN_SMALL_NORMAL);
	driveHoldHeading(3000, -60, currentHeading, DRIVE_BACK_SONAR);
	drivePower(0);
	delay(100);
	for (int i = 0; i < 5; i++)
	{
		wait1Msec(50);
		if (largestBlock != 0) break;
		//largestBlockX = 160;
	}

	return currentHeading;
}

bool ClawFull()
{
	//TODO: if pixy thinks claw looks full, return true, else false
	if (true)
	{
		return false;
	}
	else
	{
		return true;
	}
}

float OnePass(int dir, float currentHeading, int oldColumn = 3)
{
	PixyBlock* targetBlock;

	while(1 == 1)
	{
		setClaw(0);


		if (largestBlock != 0)
		{
			writeDebugStreamLine("DFSDFSDF");
			//memcpy(targetBlock, largestBlock, sizeof(targetBlock));
		}
		else
		{
			writeDebugStreamLine("continuing?");
			continue;
		}
		/*
		writeDebugStream("old %i", oldColumn);

		int newColumn = PickHeading(oldColumn, targetBlock);

		writeDebugStream("new %i", newColumn);

		if (newColumn != oldColumn && newColumn != 0)
		{
			ColumnTransition(oldColumn, newColumn, targetBlock);
		}
		else
		{
			writeDebugStreamLine("break");
			break;
		}
		*/
		wait1Msec(50);
	}
	toggleSonar(FRONT_ON);
	driveHoldHeading(4000, 80, currentHeading, DRIVE_FORWARD_SONAR);
	setClaw(1);

	currentHeading += ROTATE_RIGHT * sgn(dir);
	driveTurnInPlace(currentHeading);

	PixyScore(currentHeading);
	toggleSonar(BACK_ON);
	currentHeading = PixyBackFromFence(dir, currentHeading);


	return currentHeading;
}

void DrivePixyStraight(int distance, int speed)
{
	resetEncoders();

	struct PID pixyHeadingPID;
	PIDInit(&pixyHeadingPID, 0.4, 0, 0);

	while (1==1)
	{
		float error = 0;
		if (largestBlock != 0)
		{
			error = largestBlock->x - 160;
		}

		float pidResult = PIDUpdate(&pixyHeadingPID, error, 0.05);

		leftDrive(speed + pidResult);
		rightDrive(speed - pidResult);

		int lEnc = getLeftEncoder();
		int rEnc = getRightEncoder();

		if (fabs(lEnc + rEnc) / 2 > distance) break;
		delay(50);
	}
}

float PixyFarZone(int dir = 1, float currentHeading)
{
	currentHeading += ROTATE_RIGHT / 3 * sgn(dir);
	driveTurnInPlace(currentHeading);
	driveHoldHeading(300, 60, currentHeading);
	currentHeading += ROTATE_LEFT / 3 * sgn(dir);
	driveTurnInPlace(currentHeading);
	driveHoldHeading(1300, 110, currentHeading, DRIVE_ENCODERS, ACCEL_FAST, 60);
	brake(1);
	setClaw(1);
	SetLiftHeight(LIFT_HIGH_HEIGHT);

	currentHeading += ROTATE_LEFT * sgn(dir);
	driveTurnInPlace(currentHeading);

	driveHoldHeading(500, 80, currentHeading, DRIVE_ENCODERS, ACCEL_FAST, 50);
	driveHoldHeading(550, 127, currentHeading, DRIVE_LINES, ACCEL_FAST, 80);
	setClaw(0);
	drivePower(127);
	wait1Msec(100);
	drivePower(50);
	wait1Msec(250);
	drivePower(0);

	return currentHeading;
}

float PixyMidFarZone(int dir = 1, float currentHeading)
{
	driveHoldHeading(1200, 110, currentHeading, DRIVE_ENCODERS, ACCEL_FAST, 60);
	setClaw(1);
	brake(1);
	SetLiftHeight(LIFT_HIGH_HEIGHT);
	wait1Msec(500);
	currentHeading += ROTATE_LEFT * sgn(dir);
	driveTurnInPlace(currentHeading);

	driveHoldHeading(200, 80, currentHeading, DRIVE_ENCODERS, ACCEL_FAST, 50);
	driveHoldHeading(550, 127, currentHeading, DRIVE_LINES, ACCEL_FAST, 80);
	setClaw(0);
	drivePower(127);
	wait1Msec(100);
	drivePower(50);
	wait1Msec(250);
	drivePower(0);

	return currentHeading;
}

float PixyMidZone(int dir = 1, float currentHeading)
{
	currentHeading += ROTATE_LEFT / 3 * sgn(dir);
	brake(1);
	driveTurnInPlace(currentHeading);
	driveHoldHeading(200, 60, currentHeading);
	currentHeading += ROTATE_RIGHT / 3 * sgn(dir);
	driveTurnInPlace(currentHeading);
	driveHoldHeading(1400, 110, currentHeading, DRIVE_ENCODERS, ACCEL_FAST, 60);

	setClaw(1);
	brake(1);
	SetLiftHeight(LIFT_HIGH_HEIGHT);

	currentHeading += ROTATE_LEFT * sgn(dir);
	driveTurnInPlace(currentHeading);
	drivePower(0);
	wait1Msec(500);

	driveHoldHeading(200, 80, currentHeading, DRIVE_ENCODERS, ACCEL_FAST, 50);
	driveHoldHeading(550, 127, currentHeading, DRIVE_LINES, ACCEL_FAST, 80);
	setClaw(0);
	drivePower(127);
	wait1Msec(100);
	drivePower(50);
	wait1Msec(250);
	drivePower(0);

	return currentHeading;
}

float PixyMidNearZone(int dir = 1, float currentHeading)
{
	currentHeading += ROTATE_LEFT / 2.5 * sgn(dir);
	driveTurnInPlace(currentHeading);
	driveHoldHeading(400, 60, currentHeading);
	currentHeading += ROTATE_RIGHT / 2.5 * sgn(dir);
	driveTurnInPlace(currentHeading);
	driveHoldHeading(1200, 110, currentHeading, DRIVE_ENCODERS, ACCEL_FAST, 60);

	setClaw(1);
	brake(1);
	SetLiftHeight(LIFT_HIGH_HEIGHT);

	drivePower(0);
	wait1Msec(1000);

	currentHeading += ROTATE_LEFT * sgn(dir);
	driveTurnInPlace(currentHeading);

	drivePower(0);

	//driveHoldHeading(200, 80, currentHeading, DRIVE_ENCODERS, ACCEL_FAST, 50);
	driveHoldHeading(550, 127, currentHeading, DRIVE_LINES, ACCEL_FAST, 80);
	setClaw(0);
	drivePower(127);
	wait1Msec(100);
	drivePower(50);
	wait1Msec(250);
	drivePower(0);

	return currentHeading;
}

void PixyAutoNew(float currentHeading = 0)
{
int passCount = 0;
startTask(liftHeight);
setClaw(0);
SetLiftHeight(LIFT_LOW_HEIGHT);

while(1 == 1)
{
OnePass((passCount % 2)*2 -1, currentHeading);
}
}

/*
void PixyAutonNew(float currentHeading = 0)
{
	startTask(liftHeight);

	//GyroResetAngle(0);
	setClaw(0);
	SetLiftHeight(LIFT_HIGH_HEIGHT);
	int count = 0;
	int newColumn, oldColumn;
	int dir;

	while (1==1)
	{
		dir = (count % 2)*2 - 1;

		toggleSonar(BACK_ON);
		currentHeading = PixyBackFromFence(dir, currentHeading);
		drivePower(0);

		oldColumn = 4;

		delay(100);
		for (int i = 0; i < 5; i++)
		{
			wait1Msec(50);
			if (largestBlock != 0) break;
		}

		if (largestBlockX < (-6/45)*largestBlockY + 133.) newColumn = oldColumn - 2;
		else if (largestBlockX < (-2/45)*largestBlockY + 171.) newColumn = oldColumn - 1;
		else if (largestBlockX < (2/45)*largestBlockY + 203.) newColumn = oldColumn;
		else if (largestBlockX < (6/45)*largestBlockY + 227.) newColumn = oldColumn + 1;
		else newColumn = oldColumn + 2;

		if (newColumn != oldColumn && newColumn != 0)
		{
			ColumnTransition(oldColumn, newColumn, largestBlock);
		}
		toggleSonar(FRONT_ON);
		driveHoldHeading(5000, 80, currentHeading, DRIVE_FORWARD_SONAR);
		drivePower(0);
		setClaw(1);

		currentHeading += ROTATE_RIGHT * sgn(dir);
		driveTurnInPlace(currentHeading);

		PixyScore(currentHeading);

		count++;
	}
}
*/
void PixyAuton()
{
	float currentHeading = 0;
	startTask(liftHeight);

	GyroResetAngle(0);
	SetLiftHeight(LIFT_LOW_HEIGHT);
	setClaw(0);

	// Delete Later
	SetLiftHeight(LIFT_HIGH_HEIGHT);
	// ---

	while (1==1)
	{
		toggleSonar(BACK_ON);
		currentHeading = PixyBackFromFence(1, currentHeading);

		delay(100);
		for (int i = 0; i < 5; i++)
		{
			wait1Msec(50);
			if (largestBlock != 0) break;
		}

		// Far-Right
		if (largestBlockX > 180) currentHeading = PixyFarZone(1, currentHeading);
		// Mid-Right
		else if (largestBlockX > 150) currentHeading = PixyMidFarZone(1, currentHeading);
		// Mid
		else if (largestBlockX > 120) currentHeading = PixyMidZone(1, currentHeading);
		// Mid-Left
		else if (largestBlockX > 90) currentHeading = PixyMidNearZone(1, currentHeading);
		else
		{
			currentHeading = PixyMidZone(1, currentHeading);
		}

		// Come back around
		currentHeading = PixyBackFromFence(-1, currentHeading);

		delay(100);
		for (int i = 0; i < 5; i++)
		{
			wait1Msec(50);
			if (largestBlock != 0) break;
		}

		// Far
		if (largestBlockX < 125) currentHeading = PixyFarZone(-1, currentHeading);
		// Mid-Far
		else if (largestBlockX < 160) currentHeading = PixyMidFarZone(-1, currentHeading);
		// Mid
		else if (largestBlockX < 220) currentHeading = PixyMidZone(-1, currentHeading);
		// Mid-Near
		else if (largestBlockX < 270) currentHeading = PixyMidNearZone(-1, currentHeading);
		else
		{
			currentHeading = PixyMidZone(-1, currentHeading);
		}

		drivePower(0);
	}
	setClaw(1);
}
