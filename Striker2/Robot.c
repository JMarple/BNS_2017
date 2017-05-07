#ifndef __ROBOT_C__
#define __ROBOT_C__

#define LIFT_LOW_HEIGHT 3650
#define LIFT_LITTLE_BIT_HEIGHT 3000
#define LIFT_MID_CUBE_HEIGHT 2500
#define LIFT_SIDE_PUSH_HEIGHT 2000
#define LIFT_MID_HEIGHT 1900
#define LIFT_NEARLY_HIGH_HEIGHT 1700
#define LIFT_HIGH_HEIGHT 1450

#define PID_INPLACE_TURN_NORMAL 0x00
#define PID_INPLACE_TURN_PUSHER 0x01
#define PID_INPLACE_TURN_SMALL_NORMAL 0x02
#define PID_INPLACE_TURN_WITH_CUBE 0x03
#define PID_INPLACE_HANGING 0x04
#define PID_INPLACE_TURN_PUSHER_FALLING 0x05
#define PID_INPLACE_TURN_WITH_DRAGGING_STARS 0x06
#define PID_INPLACE_NUDGE 0x07

// Function Prototypes
void SetMotorLinear(tMotor mot, float dutyCycle);

void leftDrive(int power)
{
  SetMotorLinear(driveL1, power/127.0);
  SetMotorLinear(driveL2, power/127.0);
  SetMotorLinear(driveL3, power/127.0);
}

void rightDrive(int power)
{
  SetMotorLinear(driveR1, power/127.0);
  SetMotorLinear(driveR2, power/127.0);
  SetMotorLinear(driveR3, power/127.0);
}

void setHangingLock(int value)
{
  SensorValue[hangingLock] = value;
}

void drivePower(int power)
{
  leftDrive(power);
  rightDrive(power);
}

void liftPower(int power)
{
  motor[liftL1] = motor[liftR1] = power;
}

int getLeftEncoder()
{
  return SensorValue[leftEncoder];
}

int getRightEncoder()
{
  return SensorValue[rightEncoder];
}

int getLeftLine()
{
  return SensorValue[lLine];
}

int getRightLine()
{
  return SensorValue[rLine];
}

int getBackSonar()
{
  int sonarVal = SensorValue[backSonar];
  return sonarVal < 10000 ? sonarVal : -1;
}

int getFrontSonar()
{
  int sonarVal = SensorValue[frontSonar];
  return sonarVal < 10000 ? sonarVal : -1;
}

void setClaw(int value)
{
  SensorValue[intake] = value;
}

void setPusher(int value)
{
  SensorValue[pusher] = value;
}

void resetEncoders()
{
  SensorValue[leftEncoder] = 0;
  SensorValue[rightEncoder] = 0;
}

float getLiftHeight()
{
  return SensorValue(pot);
}

enum SonarLocation
{
  FRONT_ON,
  BACK_ON
};

void toggleSonar(SonarLocation sonar)
{
  if(sonar == FRONT_ON)
  {
    SensorType[dgtl10] = sensorNone;
    SensorType[dgtl6] = sensorSONAR_TwoPins_raw;
  }
  else if(sonar == BACK_ON)
  {
    SensorType[dgtl6] = sensorNone;
    SensorType[dgtl10] = sensorSONAR_TwoPins_raw;
  }
}

void GyroCalibration()
{
  // Wait for first packet to come
  while (packetCount <= 50){;}
  gyroCalibYaw = (int)(gyroYaw / 50.0);

  writeDebugStreamLine("Calib yaw value = %f", gyroCalibYaw);
}

void GyroResetAngle(float value)
{
  heading.data[0] = 1;
  heading.data[1] = 0;
  heading.data[2] = 0;
  heading.data[3] = 0;
  packetCount = 0;
  previousGyroYaw = 0;
}

float GyroGetAngle()
{
  return gyroYaw;
}

float GyroGetRate()
{
  return gyroYawRate;
}

struct PID
{
  // Proportional, Integral, Derivative constants.
  float kP, kI, kD;

  //
  float error, integral, derivative, previousError;

  // Max integral is how large the `integral` value can become. If left
  // as 0, the integral will not be restricted.
  float maxIntegral;

  // Max output is the maximum output value the controller can return.
  // If left as 0, the output will be restricted.
  float maxOutput;
};

// Initializes the controller to default values.
int PIDInit(struct PID* pid, float kP, float kI, float kD)
{
  pid->kP = kP;
  pid->kI = kI;
  pid->kD = kD;

  pid->previousError = 0;
  pid->integral = 0;
  pid->derivative = 0;
  pid->error = 0;

  // If max = 0, do no limit to any value.
  pid->maxIntegral = 0;
  pid->maxOutput = 0;

  return 0;
}

// Calcultes the feedback value for the PID controller.
float PIDUpdate(struct PID* pid, float error, float dT)
{
  pid->error = error;
  pid->integral += error * dT;
  pid->derivative = (error - pid->previousError) / dT;
  pid->previousError = error;

  // Ensure the integral doesn't get too large.
  if (pid->maxIntegral != 0)
  {
    if (pid->integral > pid->maxIntegral)
      pid->integral = pid->maxIntegral;

    if (pid->integral < -pid->maxIntegral)
      pid->integral = -pid->maxIntegral;
  }

  float returnValue = pid->error * pid->kP
  + pid->integral * pid->kI
  + pid->derivative * pid->kD;

  if (pid->maxOutput != 0)
  {
    if (returnValue > pid->maxOutput)
      returnValue = pid->maxOutput;

    if (returnValue < -pid->maxOutput)
      returnValue = -pid->maxOutput;
  }

  return returnValue;
}

// Ensure the duty cycle is linear from 0 -> 127 for all motors
void SetMotorLinear(tMotor mot, float dutyCycle)
{
  // Cortex's internal motor controller.
  // The relationship between motor value and pwm is perfectly
  // proportional.  Ex. 60/127 = 0.47, so the duty cycle is 47%.
  if (mot == port1 || mot == port10)
  {
    motor[mot] = (int)(dutyCycle * 127.0);
  }
  // External MC29's.
  // The relationship between motor value is linear, but goes from 6->86.
  else
  {
    if (fabs(dutyCycle) <= 0) motor[mot] = 0;
    if (dutyCycle >= 0.97) motor[mot] = 127;
    if (dutyCycle <= -0.97) motor[mot] = -127;
    else motor[mot] = (int)(82.91 * dutyCycle + 5.0854);
  }
}


int targetLiftHeight = LIFT_LOW_HEIGHT;

void SetLiftHeight(int value)
{
  targetLiftHeight = value;
}

float _liftPower = 0;

task liftHeight()
{
  struct PID liftpid;
  PIDInit(&liftpid, 0.3, 0, 0);

  while (1==1)
  {
    int angle = getLiftHeight();
    float value = PIDUpdate(&liftpid, (angle - targetLiftHeight), 0.01);
    _liftPower = value;
    liftPower(value);
    writeDebugStreamLine("%d %f %d", targetLiftHeight, _liftPower, motor[liftL1]);
    delay(10);
  }
}

float limitTo180(float degrees)
{
  while (degrees < -180)
  {
    degrees += 2*180;
  }

  while (degrees > 180)
  {
    degrees -= 2*180;
  }

  return degrees;
}

#include "pixy.c"

Pixy robotPixy;
PixyBlock* largestBlock;
PixyBlock* largestBlockInClaw;
int largestBlockX; int largestBlockY;

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

#endif
