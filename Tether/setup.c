#if !defined(SETUP_C_)
#define SETUP_C_

#include "./setup.h"

void masterSetup() {
	bMotorReflected[MASTER_DRIVE_MOTOR_R] = true;

	setSensorType(MASTER_DRIVE_ENCODER, sensorQuadEncoder);
	setSensorType(MASTER_DRIVE_ENCODER_2, sensorQuadEncoderSecondPort);
	setSensorType(MASTER_CLAW_SOLENOID, sensorDigitalOut);

	mapEncoderToMotor(MASTER_DRIVE_MOTOR_L, MASTER_DRIVE_ENCODER);

	resetMotorEncoder(MASTER_DRIVE_MOTOR_L);
	resetMotorEncoder(MASTER_LIFT_MOTOR);
}

void slaveSetup() {
	bMotorReflected[SLAVE_DRIVE_MOTOR_R] = true;
	bMotorReflected[SLAVE_DRIVE_MOTOR_R2] = true;

	setSensorType(SLAVE_CLAW_SOLENOID, sensorDigitalOut);

	setSensorType(SLAVE_GYRO, sensorAnalog);
}

#endif  // SETUP_C_
