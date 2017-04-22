#include "../gyro/gyroArray.c"

GyroArray gyroArray;

Gyro gyro1;
Gyro gyro2;
Gyro gyro3;

task background() {
	while (true) {
		update(&gyroArray);

		update(&gyro1);
		update(&gyro2);
		update(&gyro3);

		sleep(1);
	}
}

task main() {
	tSensors ports[3] = {in1, in2, in3};
	newGyroArray(&gyroArray, ports, 3);  // Set up gyroArray in analog ports 1, 2, and 3.
	calibrate(&gyroArray);  // Calibrate gyroArray.

	newGyro(&gyro1, in1);
	calibrate(&gyro1);
	newGyro(&gyro2, in2);
	calibrate(&gyro2);
	newGyro(&gyro3, in3);
	calibrate(&gyro3);

	startTask(background);  // Update gyro angle in the background.

	// Turn left 90 degrees.
	while (getDifferenceInAngleDegrees(getAngle(&gyroArray), 90.0) > 0.0) {
		motor[port2] = 127;
		motor[port3] = -127;
	}
	motor[port2] = motor[port3] = 0;
}
