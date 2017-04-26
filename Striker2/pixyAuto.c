const int PIXY_MID = 170;

//turns in place to face an object, returns the new angle
float PixyTurn(float cutoff = 30)
{
  struct PID pixyHeadingPID;
  PIDInit(&pixyHeadingPID, 0.8, 0, 0);

  float error = 0;
  error = largestBlockX - PIXY_MID;

  int dir = sgn(error);

  while (1==1)
  {

    error = largestBlockX - PIXY_MID;
    //writeDebugStreamLine("largest x: %i", largestBlockX);
    //writeDebugStreamLine("error: %i", error);

    float pidResult = PIDUpdate(&pixyHeadingPID, error, 1);

    leftDrive(pidResult);
    rightDrive(-pidResult);

    /*
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
    */

    if (abs(error) <= cutoff)
    {
      leftDrive(-5 * dir);
      rightDrive(5 * dir);
      break;
    }
    //if (abs(gyroYawRate) < 0.3) break;
    if (autoQuit == 1) break;
    delay(50);
  }
  //however much it turned, assume it was the right amount for the purposes of correcting
  return GyroGetAngle();
}

void PixyScore(float currentHeading)
{
  driveHoldHeading(200, 127, currentHeading, DRIVE_ENCODERS, ACCEL_FAST, 50, 4);
  driveHoldHeading(500, 127, currentHeading, DRIVE_LINES, ACCEL_NONE, 127, 4);
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
  //driveHoldHeading(40, 70, currentHeading, DRIVE_ENCODERS, ACCEL_SLOW, 40);
  brake(1);

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

float PixyBackToLine(float currentHeading, int dir = 1)
{
  driveHoldHeading(-300, -70, currentHeading, DRIVE_ENCODERS, ACCEL_SLOW, -40);
  SetLiftHeight(LIFT_LOW_HEIGHT);
  driveHoldHeading(-700, -70, currentHeading, DRIVE_LINES);
  driveHoldHeading(20, 70, currentHeading, DRIVE_ENCODERS, ACCEL_SLOW, 40);
  brake(1);
  drivePower(0);

  currentHeading += ROTATE_RIGHT * sgn(dir);
  driveTurnInPlace(currentHeading, PID_INPLACE_TURN_SMALL_NORMAL);

  return currentHeading;
}

void PixyAuton(float currentHeading = 0)
{
  startTask(liftHeight);
  GyroResetAngle(0);
  SetLiftHeight(LIFT_HIGH_HEIGHT);
  setClaw(0);
  toggleSonar(BACK_ON);
  int directionFacing = 1;
  float oldHeading;

  currentHeading = PixyBackFromFence(-1, currentHeading);

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
      currentHeading += ROTATE_RIGHT * 2 + directionFacing;
      driveTurnInPlace(currentHeading, PID_INPLACE_TURN_SMALL_NORMAL);
      brake(-1);
      if (directionFacing == 1) directionFacing = -1;
      else if (directionFacing == -1) directionFacing = 1;
    }
    oldHeading = currentHeading;
    currentHeading = PixyTurn();
    driveHoldHeading(10000, 60, currentHeading, DRIVE_PIXY_CLAW);
    setClaw(1);
    drivePower(-70);
    wait1Msec(600);
    currentHeading = oldHeading + ROTATE_RIGHT * sgn(directionFacing);
    driveTurnInPlace(currentHeading, PID_INPLACE_TURN_SMALL_NORMAL);
    drivePower(-70);
    wait1Msec(600);
    SetLiftHeight(LIFT_HIGH_HEIGHT);
    drivePower(70);
    wait1Msec(200);
    drivePower(0);
    waitForLiftUp(LIFT_NEARLY_HIGH_HEIGHT);
    PixyScore(currentHeading);
    currentHeading = PixyBackToLine(currentHeading, directionFacing);
    if (directionFacing == 1) directionFacing = -1;
    else if (directionFacing == -1) directionFacing = 1;
  }
}
