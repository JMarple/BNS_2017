#pragma systemFile

#if !defined(MOTOR_C_)
#define MOTOR_C_

void motorSetLinear(tMotor port, short speed) {
	if (speed == 0) {
		return 0;
	}
	short x = abs(speed);

	if (x > 127) {
		x = 127;
	}
	if (port == port1 || port == port10) {
		motor[port] = sgn(speed) * ((((0.000001115136722 * x
				- 0.0001834554708) * x + 0.01010261354) * x + 0.01924469053) * x
				+ 11.46841392);
	} else {
		motor[port] = sgn(speed) * ((((0.000001115136722 * x
				- 0.0001834554708) * x + 0.01010261354) * x + 0.01924469053) * x
				+ 11.46841392);
	}
}

void motorSetRpm(tMotor port, float rpm) {
	if (rpm == 0) {
		return 0;
	}
	float speed;
	short x = abs(rpm);

	if (x > 127) {
		x = 127;
	}
	if (port == port1 || port == port10) {
		// TODO.
	} else {
		speed = ((((((((-0.000000000005027706441 * x + 0.000000004107804923) * x
				- 0.000001406068062) * x + 0.000261791869) * x
				- 0.02878725354) * x + 1.899167243) * x - 72.91809714) * x
				+ 1498.924934) * x - 12721.41815) * x + 94754.80994;
	}
	motor[port] = sgn(rpm) * speed / nAvgBatteryLevel;
}

#endif  // MOTOR_C_
