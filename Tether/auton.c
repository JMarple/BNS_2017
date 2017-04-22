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

#endif  // AUTON_C_
