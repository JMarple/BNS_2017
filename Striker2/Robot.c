#ifndef __ROBOT_C__
#define __ROBOT_C__

#define LIFT_LOW_HEIGHT 3920
#define LIFT_MID_HEIGHT 1900
#define LIFT_HIGH_HEIGHT 1500

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
  return SensorValue[backSonar];
}

void setClaw(int value)
{
  SensorValue[intake] = value;
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






// Biquad filter is a standard 2nd-order filter
// The transfer function follows the following format:
// H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
struct BiquadFilter
{
  float b0, b1, b2, a1, a2;

  float old_input[2];
  float old_output[2];
};

// The response of the filter, default is butterworth.
enum BiquadFilterType
{
  LOWPASS,
  HIGHPASS
};

int BiquadFilterInit(
    struct BiquadFilter* filter,
    enum BiquadFilterType type,
    float sampleFreq,
    float cutoffFreq,
    float defaultValue)
{
  if (filter == 0) return -1;

  float w0 = 2 * PI * cutoffFreq / sampleFreq;
  float cosw0 = cos(w0);
  float sinw0 = sin(w0);

  float Q = 0.707;
  float alpha = sinw0 / (2 * Q);
  float b0, b1, b2, a0, a1, a2;

  // Calculates the coefficients for the transfer function.
  switch (type)
  {
    case LOWPASS:
      b0 = (1 - cosw0) / 2;
      b1 = (1 - cosw0);
      b2 = (1 - cosw0) / 2;
      a0 = 1 + alpha;
      a1 = -2 * cosw0;
      a2 = 1 - alpha;
      break;

    case HIGHPASS:
      b0 = (1 + cosw0) / 2;
      b1 = -(1 + cosw0);
      b2 = (1 + cosw0) / 2;
      a0 = 1 + alpha;
      a1 = -2 * cosw0;
      a2 = 1 - alpha;
      break;

    default:
      return -1;
  }

  // Normalize the coefficients so that a0 = 1.
  filter->b0 = b0 / a0;
  filter->b1 = b1 / a0;
  filter->b2 = b2 / a0;
  filter->a1 = a1 / a0;
  filter->a2 = a2 / a0;

  filter->old_input[0] = filter->old_input[1] = defaultValue;
  filter->old_output[0] = filter->old_output[1] = defaultValue;

  return 0;
}

float BiquadFilterSample(struct BiquadFilter* filter, float data)
{
  if (filter == 0) return -1;

  float output =
    filter->b0 * data +
    filter->b1 * filter->old_input[0] +
    filter->b2 * filter->old_input[1] -
    filter->a1 * filter->old_output[0] -
    filter->a2 * filter->old_output[1];

  filter->old_input[1] = filter->old_input[0];
  filter->old_input[0] = data;

  filter->old_output[1] = filter->old_output[0];
  filter->old_output[0] = output;

  return output;
}








struct GyroSensor
{
  // Calibration value
  int calibValue;

  // Sensor number for SensorValue[..]
  int sensorNum;

  // Current angle in degrees.
  float angle;
  float rate;
}

// Singleton assuming a single gyro sensor
struct GyroSensor gyroSen;

void GyroInit(int sensorNum)
{
  gyroSen.sensorNum = sensorNum;
  gyroSen.angle = 0;
}

void GyroCalibration()
{
  int i;
  long sum = 0;
  for (i = 0; i < 50; i++)
  {
    sum += SensorValue[gyroSen.sensorNum];
    delay(5);
  }

  gyroSen.calibValue = (int)(sum / 50.0);
}

float GyroDegPerSec()
{
  float rawAnalog = SensorValue[gyroSen.sensorNum] - gyroSen.calibValue;

  // Cortex's analog signal is 0 -> 3.3V, data is in 0 -> 4095
  float rawVoltage = rawAnalog * 3.3 / 4095.0;

  // Degrees per second.  1.1mV/dps according to datasheet.
  float dps = rawVoltage / 0.0011;

  return dps;
}

float GyroUpdate(float dT)
{
  float rate = GyroDegPerSec();

  if (abs(rate) > 2)
  {
    gyroSen.rate = rate;
    gyroSen.angle += gyroSen.rate * dT;
  }
  else
  {
    gyroSen.rate = 0;
  }

  return gyroSen.angle;
}

float GyroUpdateFiltered(struct BiquadFilter* filter, float dT)
{
  float rate = BiquadFilterSample(filter, GyroDegPerSec());

  if (abs(rate) > 2)
  {
    gyroSen.rate = rate;
    gyroSen.angle += gyroSen.rate * dT;
  }
  else
  {
    gyroSen.rate = 0;
  }

  return gyroSen.angle;
}

void GyroResetAngle(float value)
{
  gyroSen.angle = value;
}

float GyroGetRate()
{
  return gyroSen.rate;
}

float GyroGetAngle()
{
  return gyroSen.angle;
}

task GyroTask
{
  struct BiquadFilter gyroLP;
  BiquadFilterInit(&gyroLP, LOWPASS, 333.33, 40, 0);

  while (1==1)
  {
    GyroUpdateFiltered(&gyroLP, 0.003);
    wait1MSec(3);
  }
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
    if (abs(dutyCycle) <= 0) motor[mot] = 0;
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

task liftHeight()
{
  struct PID liftpid;
  PIDInit(&liftpid, 0.3, 0, 0);

  while (1==1)
  {
    int angle = getLiftHeight();
    float value = PIDUpdate(&liftpid, (angle - targetLiftHeight), 0.01);
    liftPower(value);
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

#endif
