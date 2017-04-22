#include "../gyro/gyro.c"

Gyro gyro;

task background() {
	while (true) {
		update(&gyro);

		sleep(1);
	}
}

task main() {
	newGyro(&gyro, in1);  // Set up gyro in analog port 1.
	calibrate(&gyro, 1000, 1);  // Calibrate gyro for 1 second, sampling at 1 millisecond intervals.

	startTask(background);  // Update gyro angle in the background.

	// Turn left 90 degrees.
	while (getDifferenceInAngleDegrees(getAngle(&gyro), 90.0) > 0.0) {
		motor[port2] = 127;
		motor[port3] = -127;
	}
	motor[port2] = motor[port3] = 0;
}
