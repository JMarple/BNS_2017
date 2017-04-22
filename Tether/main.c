#pragma platform(VEX2)
#pragma competitionControl(Competition)

#include "./competition.c"
#include "./setup.c"
#include "./auton.c"

void pre_auton() {
	masterSetup();
	slaveSetup();
}

task masterAuton() {
	masterSetLift(HOLD_DOWN);
	masterSetClaw(OPENED);
	masterSetDrive(FORWARD);
	sleep(1900);
	masterSetTurn(LEFT);
	sleep(200);
	masterSetDrive(LEFT);
	sleep(750);
	masterSetDrive(STOP);
	masterSetClaw(CLOSED);
	masterSetLift(UP);
	sleep(700);
	masterSetLift(HOLD_UP);
	masterSetTurn(LEFT);
	sleep(450);
	masterSetDrive(FORWARD);
	sleep(1000);
	masterSetDrive(STOP);
	masterSetClaw(OPENED);
}

task slaveAuton() {
	slaveSetLift(HOLD_DOWN);
	slaveSetClaw(OPENED);
	sleep(4000);
	slaveSetDrive(FORWARD);
	sleep(267);
}

task autonomous() {
	startTask(masterAuton);
	startTask(slaveAuton);
}

task usercontrol() {
	short masterLiftSpeed = 0;
	short slaveLiftSpeed = 0;

	bool masterClawBtnHeld = false;
	bool slaveClawBtnHeld = false;

	while (true) {
		// Master drive control.
		motor[MASTER_DRIVE_MOTOR_L] = vexRT[Ch3];
		motor[MASTER_DRIVE_MOTOR_R] = vexRT[Ch2];

		// Master drive control.
		motor[SLAVE_DRIVE_MOTOR_L] = vexRT[Ch3Xmtr2];
		motor[SLAVE_DRIVE_MOTOR_R] = motor[SLAVE_DRIVE_MOTOR_R2] = vexRT[Ch2Xmtr2];

		// Master lift control.
		if (vexRT[Btn6U]) {
			masterLiftSpeed = 127;
		} else if (vexRT[Btn6D]) {
			masterLiftSpeed = -127;
		} else if (vexRT[Btn7D]) {
			masterLiftSpeed = 0;
		} else {
			masterLiftSpeed = (masterLiftSpeed > 0) ? 10 : -10;
		}
		motor[MASTER_LIFT_MOTOR] = motor[MASTER_LIFT_MOTOR_2] = masterLiftSpeed;

		// Slave lift control.
		if (vexRT[Btn5UXmtr2]) {
			slaveLiftSpeed = 127;
		} else if (vexRT[Btn5DXmtr2]) {
			slaveLiftSpeed = -127;
		} else if (vexRT[Btn7DXmtr2]) {
			slaveLiftSpeed = 0;
		} else {
			slaveLiftSpeed = (slaveLiftSpeed > 0) ? 10 : -10;
		}
		motor[SLAVE_LIFT_MOTOR] = slaveLiftSpeed;

		// Master claw control.
		if (vexRT[Btn5U] || vexRT[Btn5D]) {
			if (!masterClawBtnHeld) {
				SensorValue[MASTER_CLAW_SOLENOID] = !SensorValue[MASTER_CLAW_SOLENOID];
				masterClawBtnHeld = true;
			}
		} else {
			masterClawBtnHeld = false;
		}

		// Slave claw control.
		if (vexRT[Btn6UXmtr2] || vexRT[Btn6DXmtr2]) {
			if (!slaveClawBtnHeld) {
				SensorValue[SLAVE_CLAW_SOLENOID] = !SensorValue[SLAVE_CLAW_SOLENOID];
				slaveClawBtnHeld = true;
			}
		} else {
			slaveClawBtnHeld = false;
		}

		sleep(5);
	}
}
