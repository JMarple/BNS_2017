const int PIXY_MID = 160;

int PickHeading(int oldColumn)
{
	int xValue, yValue, newColumn = 0;
	if (largestBlock != 0)
	{
		xValue = largestBlock->x;
		yValue = largestBlock->y;
	}
	//TODO: figure out which column this is in
	if (yValue > 4*xValue + 200) //numbers tbd
	{
		newColumn = 1;
	}
	else if (yValue > 8*xValue + 200)
	{
		newColumn = 2;
	}//etc

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
		if (largestBlock != 0)
		{
			error = largestBlock->x - PIXY_MID;
		}

		float pidResult = PIDUpdate(&pixyHeadingPID, error, 0.05);

		leftDrive(pidResult);
		rightDrive(pidResult);

		if (abs(error) <= cutoff) break;
		if (abs(error) <= 20 && abs(gyroYawRate) < 0.3) break;
		delay(50);
	}
	//however much it turned, assume it was the right amount for the purposes of correcting
	return GyroGetAngle();
}

void ColumnTransition(int oldColumn, int newColumn)
{
	float columnDist = 10; //TODO: figure out what this number should be

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
	while(ClawFull() == false && bIfiAutonomousMode == true)
	{
		int newColumn = PickHeading(oldColumn);
		if (newColumn != oldColumn && newColumn != 0)
		{
			ColumnTransition(oldColumn, newColumn);
		}
		else
			break;
	}
	driveHoldHeading(3000, 80, currentHeading, DRIVE_FORWARD_SONAR);
	setClaw(1);

	currentHeading += ROTATE_RIGHT * sgn(dir);
	driveTurnInPlace(currentHeading);

	PixyScore(currentHeading);
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
