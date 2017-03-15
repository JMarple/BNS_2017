#ifndef __AUTON_C__
#define __AUTON_C__

#include "Robot.c"

#define DRIVE_ENCODERS 0
#define DRIVE_LINES 1
#define DRIVE_BACK_SONAR 2
#define DRIVE_BACK_SONAR_FORWARD 3
#define DRIVE_FORWARD_SONAR 4
#define LINE_DETECT_VALUE 1900

#define ACCEL_NONE 0
#define ACCEL_SLOW 2
#define ACCEL_FAST 5

#define TYPE_FORWARD 0
#define TYPE_BACKWARD 1

#define ROTATE_RIGHT -90
#define ROTATE_LEFT 90

void brake(int dir)
{
  drivePower(-20*dir);
  wait1Msec(250);
  drivePower(0);
}

void driveStraight(int distance, int speed)
{
  resetEncoders();

  while (1==1)
  {
    leftDrive(speed);
    rightDrive(speed);

    int lEnc = getLeftEncoder();
    int rEnc = getRightEncoder();

    if (fabs(lEnc + rEnc) / 2 > distance) break;
  }
}

void driveCorrectedSmoothTurn(
	float targetAngle, int medianSpeed, int maxPIDOutput = 70)
{
  struct PID posPID;

  PIDInit(&posPID, 2, 0, 0.18);
  posPID.maxOutput = maxPIDOutput;

  while (1==1)
  {
    // Break if close enough to target angle
    float posError =
      limitTo180(targetAngle - GyroGetAngle());

    float speed = PIDUpdate(&posPID, posError, 0.01);

    if (abs(posError) <= 3) break;

    leftDrive(medianSpeed - speed);
    rightDrive(medianSpeed + speed);

    delay(10);
  }
}

void driveTurnInPlace(float targetAngle, int pidTuning = PID_INPLACE_TURN_NORMAL)
{
  struct PID posPID;

  if (pidTuning == PID_INPLACE_TURN_NORMAL) PIDInit(&posPID, 2, 0, 0.18);
  else if (pidTuning == PID_INPLACE_TURN_SMALL_NORMAL) PIDInit(&posPID, 2, 0, 0.15);
  else if (pidTuning == PID_INPLACE_TURN_PUSHER) PIDInit(&posPID, 2, 0, 0.35);
  posPID.maxOutput = 90;

  while (1==1)
  {
    // Break if close enough to target angle
    float posError =
      limitTo180(targetAngle - GyroGetAngle());

    float speed = PIDUpdate(&posPID, posError, 0.01);

    if (abs(posError) <= 3) break;

    leftDrive(-speed);
    rightDrive(speed);

    delay(10);
  }
}

int driveSmoothTurn(
	float speed,
	float targetAngularVelocity,
	float targetAngle)
{
  struct PID velPID;
  PIDInit(&velPID, 0.02, 0, 0);
  speed /= 3;
  int speedSgn = sgn(speed);

  while (1==1)
  {
    // Break if close enough to target angle
    float posError =
      limitTo180(targetAngle - GyroGetAngle());

    if (abs(posError) <= 5) break;

    // Correct for velocity differences.
    float velError = (targetAngularVelocity - GyroGetRate()) * sgn(targetAngularVelocity) * speedSgn;
    speed += PIDUpdate(&velPID, velError, 0.01);

    // Drive Motors
    if (sgn(targetAngularVelocity) == speedSgn)
    {
      leftDrive(-20 * sgn(speed));
      rightDrive(speed);
    }
    else
    {
      leftDrive(speed);
      rightDrive(-20*sgn(speed));
    }

    delay(10);
  }

  return 0;
}

void turnLeft(float speed, float angVel, float targetAngle, int type = 0)
{
  if (type == TYPE_FORWARD)
    driveSmoothTurn(fabs(speed), fabs(angVel), targetAngle);
  else if (type == TYPE_BACKWARD)
    driveSmoothTurn(-fabs(speed), fabs(angVel), targetAngle);
}

void turnRight(float speed, float angVel, float targetAngle, int type = 0)
{
  if (type == TYPE_FORWARD)
    driveSmoothTurn(fabs(speed), -fabs(angVel), targetAngle);
  else if (type == TYPE_BACKWARD)
    driveSmoothTurn(-fabs(speed), -fabs(angVel), targetAngle);
}

int driveHoldHeading(
	int distance,
	int maxSpeed,
	float targetAngle,
	int type = TYPE_FORWARD,
	int accel_type = ACCEL_NONE,
	int accel_startspeed = 0,
	int offset = 0)
{
  if (distance < 0) distance = -distance;

  resetEncoders();

  struct PID headingpid;
  PIDInit(&headingpid, 2, 0, 0);

  int speed = maxSpeed;

  if (accel_type != ACCEL_NONE)
    speed = accel_startspeed;

  while (1==1)
  {
    float error = limitTo180(targetAngle - GyroGetAngle());
    float pidResult = PIDUpdate(&headingpid, error, 0.01);

    // Acceleration code.
    if (accel_type != ACCEL_NONE)
    {
      if (accel_startspeed < maxSpeed)
      {
        speed += accel_type;
        if (speed > maxSpeed) speed = maxSpeed;
      }
      else
      {
        speed -= accel_type;
        if (speed < maxSpeed) speed = maxSpeed;
      }
    }

    leftDrive(speed - pidResult - offset);
    rightDrive(speed + pidResult + offset);

    int lEnc = getLeftEncoder();
    int rEnc = getRightEncoder();

    if (type == DRIVE_ENCODERS || type == DRIVE_LINES)
    {
      if (fabs(lEnc + rEnc) / 2 > distance) break;
    }

    if (type == DRIVE_LINES)
    {
      int leftLine = getLeftLine();
      int rightLine = getRightLine();

      if (leftLine < LINE_DETECT_VALUE || rightLine < LINE_DETECT_VALUE)
      {
        writeDebugStreamLine("Line cutoff val = %d d", leftLine, rightLine);
        break;
      }

    }

    if (type == DRIVE_BACK_SONAR)
    {
      int sonarVal = getBackSonar();
      if (sonarVal < distance && sonarVal > 0) break;
    }

    if (type == DRIVE_BACK_SONAR_FORWARD)
    {
      int sonarVal = getBackSonar();
      if (sonarVal > distance && sonarVal > 0) break;
    }

    if (type == DRIVE_FORWARD_SONAR)
    {
      int sonarVal = getFrontSonar();
      if (sonarVal > distance && sonarVal > 0) break;
    }

    delay(10);
  }

  return 0;
}

#endif
