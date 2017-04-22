#if !defined(AUTON_C_)
#define AUTON_C_

#include "./setup.h"

#define FORWARD  	127
#define BACKWARD 	-127
#define LEFT     	127
#define RIGHT    	-127
#define STOP     	0
#define UP       	127
#define DOWN     	-127
#define HOLD_UP  	10
#define HOLD_DOWN	-10
#define OPENED   	0
#define CLOSED   	1

void masterSetDrive(signed char left, signed char right) {
	motor[MASTER_DRIVE_MOTOR_L] = left;
	motor[MASTER_DRIVE_MOTOR_R] = right;
}

void slaveSetDrive(signed char left, signed char right) {
	motor[SLAVE_DRIVE_MOTOR_L] = left;
	motor[SLAVE_DRIVE_MOTOR_R] = motor[SLAVE_DRIVE_MOTOR_R2] = right;
}

void masterSetDrive(signed char speed) {
	masterSetDrive(speed, speed);
}

void slaveSetDrive(signed char speed) {
	slaveSetDrive(speed, speed);
}

void masterSetTurn(signed char speed) {
	masterSetDrive(-speed, speed);
}

void slaveSetTurn(signed char speed) {
	slaveSetDrive(-speed, speed);
}

void masterSetLift(signed char speed) {
	motor[MASTER_LIFT_MOTOR] = motor[MASTER_LIFT_MOTOR_2] = speed;
}

void slaveSetLift(signed char speed) {
	motor[SLAVE_LIFT_MOTOR] = speed;
}

void masterSetClaw(char state) {
	SensorValue[MASTER_CLAW_SOLENOID] = state;
}

void slaveSetClaw(char state) {
	SensorValue[SLAVE_CLAW_SOLENOID] = state;
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

//kp for gyro turns
const float kpg = 0.15, kig = 0.04, kdg = 0.6;

//gyro turns
void gyroTurn(int degree, int maxPower)
{
	const float cmg = 4.0;
	const float valueToDegree = 10.5;
	int value = degree * valueToDegree;
	int error = degree;
	int lastError = error;
	int integralG = 0;
	int dg = error - lastError;
	int waitTime = 25;
	float leftPower;
	float rightPower;

	int countlim = sqrt(abs(degree)) * cmg;

	for(int i=0; i<countlim; i++)
	{
		masterSetDrive((error * kpg) + (integralG * kig) + (dg * kdg), (error * kpg) + (integralG * kig) + (dg * kdg));

		if(abs(leftPower) > maxPower)
		{
			leftPower = maxPower * sgn(leftPower);
		}
		if(abs(rightPower) > maxPower)
		{
			rightPower = maxPower * sgn(rightPower);
		}

		masterSetDrive(-leftpower, rightPower);

		wait1Msec(waitTime);

		lastError = error;
		error = value - SensorValue(MASTER_GYRO);

		integralG += error;
		dg = error - lastError;
		if(abs(error) > 100)
		{
			integralG = 0;
			dg = 0;
		}
	}
	masterSetDrive(0);
}

#endif  // AUTON_C_
